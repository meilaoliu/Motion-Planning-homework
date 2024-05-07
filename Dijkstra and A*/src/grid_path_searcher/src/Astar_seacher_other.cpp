#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

//初始化grid map
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);//这个应该是左下边界
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;//计算出整个map的最大index

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

    //二重指针数组，这样理解：
    // int[5]是一个一维数组元素是int数，是一条线，
    // int*[5]是一个二维数组，元素是一维数组指针，首地址是第1条线的地址，+1访问下一条线，
    // int**[5]是3维数组，元素是2维数组指针，是一个矩形空间，首地址是第一个平面的地址，+1访问下一个平面
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];//存放所有grid node 的数组
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
    ROS_DEBUG("\nheuristic: %d, tie breaker: %d", heu_, tie_breaker_);
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

//idx to coord
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt)
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */
    //A*每次找26个neighbor node，判断是否occupied，如果不是，计算edge cost(g)，并加入到open list中
    Eigen::Vector3i tmp_index = currentPtr->index;
    Eigen::Vector3i tmp_lower, tmp_upper;
    tmp_lower <<    min( max( int( (tmp_index(0) - 1) ), 0), GLX_SIZE - 1),
                    min( max( int( (tmp_index(1) - 1) ), 0), GLY_SIZE - 1),
                    min( max( int( (tmp_index(2) - 1) ), 0), GLZ_SIZE - 1);
    tmp_upper <<    min( max( int( (tmp_index(0) + 1) ), 0), GLX_SIZE - 1),
                    min( max( int( (tmp_index(1) + 1) ), 0), GLY_SIZE - 1),
                    min( max( int( (tmp_index(2) + 1) ), 0), GLZ_SIZE - 1);
    for(int i = tmp_lower(0); i <= tmp_upper(0); i++){
        for(int j = tmp_lower(1); j <= tmp_upper(1); j++){
            for(int k = tmp_lower(2); k <= tmp_upper(2); k++){
                //free节点才加入扩展
                Eigen::Vector3i neighbor_idx(i,j,k);
                if(isFree(neighbor_idx) && neighbor_idx != tmp_index) {//只加入free node，且自己不加
                    neighborPtrSets.emplace_back(GridNodeMap[i][j][k]);
                    edgeCostSets.emplace_back(
                            sqrt(
                                std::pow((i - currentPtr->index(0)), 2)
                                + std::pow((j - currentPtr->index(1)), 2)
                                + std::pow((k - currentPtr->index(2)), 2))
                            );
                }
            }
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    if(!node1 || !node2) {
        ROS_ERROR("Invalid node pointer");
        return std::numeric_limits<double>::infinity(); // 或其他错误处理方式
    }
    double heuristic_value;
    if( heu_ == ZERO ) {
        return 0;
    } else if ( heu_ == L1 ) {
        heuristic_value = (node1->coord-node2->coord).lpNorm<1>();
    } else if ( heu_ == L2 ) {
        heuristic_value = (node1->coord-node2->coord).lpNorm<2>();
    } else if ( heu_ == DIAGONAL ) {//对角diagonal暂时先返回2D的diagonal的结果
        double dx = fabs(node1->coord.x() - node2->coord.x());
        double dy = fabs(node1->coord.y() - node2->coord.y());
        double dz = fabs(node1->coord.z() - node2->coord.z());
        double tmp_min = std::min(min(dx, dy), dz);
        heuristic_value = (dx + dy + dz) + (sqrt3_ - 3) * tmp_min;//助教给的答案公式
    }

    switch( tie_breaker_ ){
        case RULE_1:
            if(heu_== L1)
                tb_rule1_p_ = 1.0 / (Eigen::Vector3d(GLX_SIZE, GLY_SIZE, GLZ_SIZE)).lpNorm<1>();// 1.0/可能path的最大cost
            else if(heu_== L2) {
                tb_rule1_p_ = 1.0 / (Eigen::Vector3d(GLX_SIZE, GLY_SIZE, GLZ_SIZE)).lpNorm<2>();
            } else if(heu_== DIAGONAL) {
                tb_rule1_p_ = 1.0 / (GLX_SIZE + GLY_SIZE + GLZ_SIZE) + (sqrt3_ - 3) * std::min(min(GLX_SIZE, GLY_SIZE), GLZ_SIZE);
            }
            heuristic_value = heuristic_value * (1 + tb_rule1_p_);
//            ROS_DEBUG_STREAM("tie breaker RULE_0 p: " << tb_rule1_p_);//L2 tb_rule1_p_=0.0133333
            break;
        case RULE_2://未来可能的实现
            break;
        case RULE_3:
            break;
        case RULE_4:
            break;
        case RULE_NONE: default:
            break;
    }
    return heuristic_value;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = startPtr -> gScore + getHeu(startPtr,endPtr);
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );//默认按照key升序排序
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    double tentative_gScore;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    //计算graph中所有的node到start_pt的heuristic

    //将start node赋予地图(因为是刚才new出来的，不是直接从地图里面取出来的)
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)] = startPtr;
    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        //弹出一个就要找到他所有的未被扩展的邻居
//        ROS_DEBUG("\nhere3");
        currentPtr = openSet.begin()->second;
        openSet.erase(openSet.begin());
        currentPtr->id = -1;//mark as visited

        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the succetion(找到currentPtr的neighbor nodes)
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */
        //在这个循环里面，id只会为0/1，为-1的都是遍历完之后被erase掉了
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            tentative_gScore = currentPtr->gScore + edgeCostSets[i];
//            ROS_DEBUG("\nneighborPtr->id: %d", neighborPtr->id);
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            if(neighborPtr -> id == 0 ){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */

                if(neighborPtr -> gScore == inf) {
                    neighborPtr->cameFrom = currentPtr;
                    neighborPtr -> gScore = tentative_gScore;
                    neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr, endPtr); //getHeu(startPtr,neighborPtr);
                    neighborPtr -> id = 1;//标记为expanded
                    openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) );//加入到open list中，注意，这里需要使用fscore，而不是gscore，如果是gscore，就成了Dijkstra了
//                    ROS_DEBUG("\nneighborPtr -> gScore is inf");
                    continue;//这里可以直接退出本轮循环，因为下面的条件不会满足
                } else {
                    ROS_DEBUG_STREAM("\nA* here neighborPtr -> gScore is not inf");
                }
            }
            //这里不能取等，去等了如果前面在找neighbors时没有去掉本身，就会陷入自己来自于自己的bug
            if(tentative_gScore < neighborPtr-> gScore){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                neighborPtr -> cameFrom = currentPtr;
                neighborPtr -> gScore = tentative_gScore;
                neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr, endPtr);

            }
            //不满足条件的就是在close set中，不用处理，在下次的while循环中会被erase掉
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath()
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    Vector3d end_pt   = gridIndex2coord(goalIdx);
    GridNodePtr tmpPtr = terminatePtr;
//    ROS_DEBUG_STREAM("terminatePtr->gscore: " << tmpPtr->gScore);

    while(tmpPtr->cameFrom != nullptr) { //这两种判断方式都可以，gscore为0或者cameFrome为空
//    while(tmpPtr->gScore != 0) {
        gridPath.emplace_back(tmpPtr);
        tmpPtr = tmpPtr->cameFrom;
    }
    ROS_DEBUG_STREAM("\nhas found start goal, coord: " << tmpPtr->coord.transpose());
    ROS_DEBUG_STREAM("\ninverse path:, gridPath.size()" << gridPath.size() );

    for (auto ptr: gridPath) {
        path.push_back(ptr->coord);
        ROS_DEBUG_STREAM("" << ptr->coord.transpose());
    }
        
    reverse(path.begin(),path.end());

    return path;
}

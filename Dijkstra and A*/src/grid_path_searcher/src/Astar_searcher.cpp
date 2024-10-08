#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
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
                if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                // if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

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
   //mode1 路径可以斜着走 cost用网络索引计算
//    for (int dx = -1; dx <= 1; dx++) {
//         for (int dy = -1; dy <= 1; dy++) {
//             for (int dz = -1; dz <= 1; dz++) {
//                 if (dx == 0 && dy == 0 && dz == 0)
//                     continue; // Skip the current node itself
                
//                 Eigen::Vector3i neighborIndex = currentPtr->index + Eigen::Vector3i(dx, dy, dz);

//                 // Check if the neighbor is within grid bounds and not an obstacle
//                 if (isFree(neighborIndex)) {
//                     GridNodePtr neighborPtr = GridNodeMap[neighborIndex(0)][neighborIndex(1)][neighborIndex(2)];
//                     neighborPtrSets.push_back(neighborPtr);
                    
//                     // Calculate the cost from current node to the neighbor
//                     double cost = std::sqrt(dx * dx + dy * dy + dz * dz); // Euclidean distance
//                     edgeCostSets.push_back(cost);
//                 }
//             }
//         }
//     }

   //mode1 路径可以斜着走 cost用坐标计算
   for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
        for (int dz = -1; dz <= 1; dz++) {
            if (dx == 0 && dy == 0 && dz == 0)
                continue; // Skip the current node itself
            
            Eigen::Vector3i neighborIndex = currentPtr->index + Eigen::Vector3i(dx, dy, dz);

            // Check if the neighbor is within grid bounds and not an obstacle
            if (isFree(neighborIndex)) {
                GridNodePtr neighborPtr = GridNodeMap[neighborIndex(0)][neighborIndex(1)][neighborIndex(2)];
                neighborPtrSets.push_back(neighborPtr);
                
                // Calculate the cost from current node to the neighbor using spatial coordinates
                Eigen::Vector3d currentCoord = currentPtr->coord;
                Eigen::Vector3d neighborCoord = gridIndex2coord(neighborIndex);
                double cost = (currentCoord - neighborCoord).norm(); // Euclidean distance in space
                edgeCostSets.push_back(cost);
            }
        }
    }
}


   //mode2 路径不开眼斜着走
//    static const int directions[6][3] = {
//         {1, 0, 0}, {-1, 0, 0}, // x轴正向和反向
//         {0, 1, 0}, {0, -1, 0}, // y轴正向和反向
//         {0, 0, 1}, {0, 0, -1}  // z轴正向和反向
//     };

//     for (int i = 0; i < 6; i++) {
//         Eigen::Vector3i dir(directions[i][0], directions[i][1], directions[i][2]);
//         Eigen::Vector3i neighborIndex = currentPtr->index + dir;

//         // Check if the neighbor is within grid bounds and not an obstacle
//         if (isFree(neighborIndex)) {
//             GridNodePtr neighborPtr = GridNodeMap[neighborIndex(0)][neighborIndex(1)][neighborIndex(2)];
//             neighborPtrSets.push_back(neighborPtr);
            
//             // Calculate the cost from current node to the neighbor
//             double cost = 1.0; // Since we're moving in one axis at a time, the cost is always 1.
//             edgeCostSets.push_back(cost);
//         }
//     }


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
   double fcore = 0 ;
   double epsilon = 0.01;
   double dx = abs(node1->coord(0)-node2->coord(0));
   double dy = abs(node1->coord(1)-node2->coord(1));
   double dz = abs(node1->coord(2)-node2->coord(2));
   double heuristic_result = 0.0; // 初始化启发式计算结果
   //double g = node1->gScore; // 从起点到当前节点的实际成本

    switch(heuristic_type_) {
    case EUCLIDEAN: // 欧式距离
        heuristic_result = std::sqrt(dx * dx + dy * dy + dz * dz);
        break;
    case MANHATTAN: // 曼哈顿距离
         heuristic_result = dx + dy + dz;
        break;
    case DIAGONAL: {
        // double diagonal_min = std::min({dx, dy, dz});
        // heuristic_result = (dx + dy + dz) + (std::sqrt(3.0) - 3) * diagonal_min;
         double dmin = min( min(dx, dy), dz);
        double dmax = max(max(dx, dy), dz);
        double dmid = dx + dy + dz - dmin - dmax;
        heuristic_result = (sqrt(3) - sqrt(2))*dmin + (sqrt(2) - 1)*dmid + dmax;
        break;
    }
    case DIJKSTRA: // Dijkstra 算法，不使用启发函数
        heuristic_result = 0;
        break;
    default:
        heuristic_result = std::sqrt(dx * dx + dy * dy + dz * dz); // 默认使用欧几里得距离
        break;
    }
    // 加入Tie-breakingh(n) = (1 + ε) * h(n) + ε * g(n)
    //heuristic_result = (1 + epsilon) * heuristic_result + epsilon * g; 发生内存错误
    // 加入Tie-breakingh(n) = (1 + ε) * h(n)
     heuristic_result = (1 + epsilon) * heuristic_result;
    return  factor_*heuristic_result; // 返回计算结果
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    // 将起点和终点坐标转换为网格索引
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    //将网格索引转换回空间坐标
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    //初始化起点和终点节点 包含了起点和终点的准确位置和网格索引信息。
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );

    // closeSet.clear();
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    
    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to c
        losed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
 

        auto lowestFscoreNode = openSet.begin(); // 获取最低fScore的节点
        currentPtr = lowestFscoreNode->second; // 访问节点指针
        openSet.erase(lowestFscoreNode); // 从openSet中移除
        currentPtr->id = -1;
        // closeSet.insert({currentPtr->index, currentPtr}); // 加入closeSet


        
        // if the current node is the goal .
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
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
           GridNodePtr neighborPtr  = neighborPtrSets [i];//当前邻居节点
           double current_gScore = currentPtr->gScore + edgeCostSets[i];// 计算当前gScore得分


            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
               neighborPtr->cameFrom = currentPtr;
               neighborPtr->gScore = current_gScore;
               neighborPtr->fScore = current_gScore+getHeu(neighborPtr, endPtr);//计算邻居节点的fcore
               neighborPtr->id = 1;//标记加入openlist中
               openSet.insert({neighborPtr->fScore, neighborPtr}); // 加入openlist中
                continue;
            }
            else if(neighborPtr->id == 1 && current_gScore < neighborPtr->gScore){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */ // 更新节点信息
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = current_gScore;
                neighborPtr->fScore = current_gScore + getHeu(neighborPtr, endPtr); // 重新计算fScore

                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                continue;
            }
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
   GridNodePtr currentNode = terminatePtr; // 从终点开始回溯
      // 回溯路径
    while (currentNode != nullptr) { // 如果当前节点不为空，则继续
        gridPath.push_back(currentNode); // 将当前节点加入到路径中
        currentNode = currentNode->cameFrom; // 移动到当前节点的父节点
    }


    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}
#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"
#include <cmath>

enum HeuristicType {
    EUCLIDEAN,
    MANHATTAN,
    DIAGONAL,
    DIJKSTRA // 启发函数类型
};

class AstarPathFinder
{	
	private:
	  double factor_; // 用于存储动态配置中设置的因子
	protected:
		uint8_t * data;
		GridNodePtr *** GridNodeMap;
		Eigen::Vector3i goalIdx;
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		HeuristicType heuristic_type_; // 添加启发函数类型成员变量

		double resolution, inv_resolution;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;

		GridNodePtr terminatePtr;
		
		std::multimap<double, GridNodePtr> openSet;
		std::multimap<double, GridNodePtr> closeSet;

		double getHeu(GridNodePtr node1, GridNodePtr node2);
		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

    	bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;
		
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

	public:
	    AstarPathFinder() : heuristic_type_(EUCLIDEAN), factor_(1.0) {}; // 默认使用欧几里得启发函数
		//AstarPathFinder(){};
		~AstarPathFinder(){};

        void setHeuristicType(HeuristicType type) {
            heuristic_type_ = type; // 设置启发函数类型
        }
		void setFactor(double factor) { factor_ = factor; } // 设置因子的方法
		
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		void resetGrid(GridNodePtr ptr);
		void resetUsedGrids();

		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		void setObs(const double coord_x, const double coord_y, const double coord_z);

		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		std::vector<Eigen::Vector3d> getPath();
		std::vector<Eigen::Vector3d> getVisitedNodes();
};

#endif
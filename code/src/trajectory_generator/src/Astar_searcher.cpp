#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void Astarpath::begin_grid_map(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GRID_X_SIZE = max_x_id;
  GRID_Y_SIZE = max_y_id;
  GRID_Z_SIZE = max_z_id;
  GLYZ_SIZE = GRID_Y_SIZE * GRID_Z_SIZE;
  GLXYZ_SIZE = GRID_X_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  Map_Node = new MappingNodePtr **[GRID_X_SIZE];
  for (int i = 0; i < GRID_X_SIZE; i++) {
    Map_Node[i] = new MappingNodePtr *[GRID_Y_SIZE];
    for (int j = 0; j < GRID_Y_SIZE; j++) {
      Map_Node[i][j] = new MappingNodePtr[GRID_Z_SIZE];
      for (int k = 0; k < GRID_Z_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        Map_Node[i][j][k] = new MappingNode(tmpIdx, pos);
      }
    }
  }
}

void Astarpath::resetGrid(MappingNodePtr ptr) {
  ptr->id = 0;
  ptr->Father = NULL;
  ptr->g_score = inf;
  ptr->f_score = inf;
}

void Astarpath::resetUsedGrids() {
  for (int i = 0; i < GRID_X_SIZE; i++)
    for (int j = 0; j < GRID_Y_SIZE; j++)
      for (int k = 0; k < GRID_Z_SIZE; k++)
        resetGrid(Map_Node[i][j][k]);
}

void Astarpath::set_barrier(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GRID_Z_SIZE || idx_x == GRID_X_SIZE ||
      idx_y == GRID_Y_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] = 1;
  else {
    data[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GRID_Z_SIZE + idx_z] = 1;
  }
}

vector<Vector3d> Astarpath::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GRID_X_SIZE; i++)
    for (int j = 0; j < GRID_Y_SIZE; j++)
      for (int k = 0; k < GRID_Z_SIZE; k++) {
        // if(Map_Node[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (Map_Node[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(Map_Node[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %zu", visited_nodes.size());
  return visited_nodes;
}

Vector3d Astarpath::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i Astarpath::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GRID_X_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GRID_Y_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GRID_Z_SIZE - 1);

  return idx;
}

Vector3i Astarpath::c2i(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GRID_X_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GRID_Y_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GRID_Z_SIZE - 1);

  return idx;
}

Eigen::Vector3d Astarpath::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool Astarpath::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

bool Astarpath::is_occupy(const Eigen::Vector3i &index) {
  return isOccupied(index(0), index(1), index(2));
}

inline bool Astarpath::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool Astarpath::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GRID_X_SIZE && idx_y >= 0 && idx_y < GRID_Y_SIZE &&
          idx_z >= 0 && idx_z < GRID_Z_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] == 1));
}

inline bool Astarpath::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GRID_X_SIZE && idx_y >= 0 && idx_y < GRID_Y_SIZE &&
          idx_z >= 0 && idx_z < GRID_Z_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] < 1));
}

inline void Astarpath::AstarGetSucc(MappingNodePtr currentPtr,
                                          vector<MappingNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i Idx_neighbor;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {

        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        Idx_neighbor(0) = (currentPtr->index)(0) + dx;
        Idx_neighbor(1) = (currentPtr->index)(1) + dy;
        Idx_neighbor(2) = (currentPtr->index)(2) + dz;

        if (Idx_neighbor(0) < 0 || Idx_neighbor(0) >= GRID_X_SIZE ||
            Idx_neighbor(1) < 0 || Idx_neighbor(1) >= GRID_Y_SIZE ||
            Idx_neighbor(2) < 0 || Idx_neighbor(2) >= GRID_Z_SIZE) {
          continue;
        }

        neighborPtrSets.push_back(
            Map_Node[Idx_neighbor(0)][Idx_neighbor(1)][Idx_neighbor(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
  }
}

double Astarpath::getHeu(MappingNodePtr node1, MappingNodePtr node2) {
  
  // 使用数字距离和一种类型的tie_breaker
  Vector3d diff = node1->coord - node2->coord;
  double heu = diff.norm();
  // 【修改4】：稍微增大 tie_breaker 以加速搜索（会牺牲一点最优性）
  double tie_breaker = 1.0 + 1.0 / 1000.0;  // 从 10000 改为 1000，加速搜索
  heu = heu * tie_breaker;
  
  return heu;
}


bool Astarpath::AstarSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();
  
  ROS_INFO("[A*] Starting search: start=[%.2f, %.2f, %.2f], end=[%.2f, %.2f, %.2f]",
           start_pt(0), start_pt(1), start_pt(2),
           end_pt(0), end_pt(1), end_pt(2));

  // start_point 和 end_point 索引
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;
  
  ROS_INFO("[A*] Grid indices: start=[%d, %d, %d], end=[%d, %d, %d]",
           start_idx(0), start_idx(1), start_idx(2),
           end_idx(0), end_idx(1), end_idx(2));

  // 【修改3】：检查终点是否在障碍物中
  if(isOccupied(end_idx)) {
    ROS_WARN("[A*] Goal is blocked! Skip path finding.");
    return false;
  }

  //start_point 和 end_point 的位置
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // 【修改2】：使用全局地图节点，而不是 new 新的节点
  // 确保索引在安全范围内
  if (start_idx(0) < 0 || start_idx(0) >= GRID_X_SIZE ||
      start_idx(1) < 0 || start_idx(1) >= GRID_Y_SIZE ||
      start_idx(2) < 0 || start_idx(2) >= GRID_Z_SIZE ||
      end_idx(0) < 0 || end_idx(0) >= GRID_X_SIZE ||
      end_idx(1) < 0 || end_idx(1) >= GRID_Y_SIZE ||
      end_idx(2) < 0 || end_idx(2) >= GRID_Z_SIZE) {
    ROS_ERROR("[A*] Start or Goal index out of bounds!");
    return false;
  }

  if(isOccupied(start_idx)) {
    ROS_WARN("[A*] Start is inside obstacle!");
    return false;
  }

  // 使用全局地图中的指针，而不是 new 新的
  MappingNodePtr startPtr = Map_Node[start_idx(0)][start_idx(1)][start_idx(2)];
  MappingNodePtr endPtr = Map_Node[end_idx(0)][end_idx(1)][end_idx(2)];

  // Openset 是通过 STL 库中的 multimap 实现的open_list
  Openset.clear();
  // currentPtr 表示 open_list 中 f（n） 最低的节点
  MappingNodePtr currentPtr = NULL;
  MappingNodePtr neighborPtr = NULL;

  // 将 Start 节点放在 Open Set 中
  startPtr->g_score = 0;
  /**
   *
   * STEP 1.1:  完成 Astarpath::getHeu
   *
   * **/
  startPtr->f_score = getHeu(startPtr, endPtr);

  

  startPtr->id = 1;
  startPtr->coord = start_pt;
  startPtr -> Father = NULL;
  Openset.insert(make_pair(startPtr->f_score, startPtr));


  double tentative_g_score;
  vector<MappingNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  /**
   *
   * STEP 1.2:  完成循环
   *
   * **/

  int iteration_count = 0;
  while (!Openset.empty()) {
    iteration_count++;
    
    // 每1000次迭代输出一次进度
    if (iteration_count % 1000 == 0) {
      ROS_INFO("[A*] Iteration %d, Open set size: %zu", iteration_count, Openset.size());
    }
    
    // 防止无限循环
    if (iteration_count > 1e7) {
      ROS_ERROR("[A*] Too many iterations (%d), aborting search!", iteration_count);
      return false;
    }
    
    //1.弹出g+h最小的节点
    auto it = Openset.begin();
    currentPtr = it->second;
    Openset.erase(it);
    
    //2.判断是否是终点
    if (currentPtr->index == goalIdx) {
      terminatePtr = currentPtr;
      ros::Time time_2 = ros::Time::now();
      double elapsed = (time_2 - time_1).toSec();
      ROS_INFO("[A*] Path found! Iterations: %d, Time: %.3f s", iteration_count, elapsed);
      if (elapsed > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", elapsed);
      return true;
    }
    
    // 将当前节点加入close set
    currentPtr->id = -1;
    
    //3.拓展当前节点
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);
    
    for(unsigned int i=0;i<neighborPtrSets.size();i++)
    {
      
      if(neighborPtrSets[i]->id==-1)
      {
         continue;
      }
      tentative_g_score=currentPtr->g_score+edgeCostSets[i];
      neighborPtr=neighborPtrSets[i];
      if(isOccupied(neighborPtr->index))
      continue;
      if(neighborPtr->id==0)
      {
        //4.填写信息，完成更新 - 新发现的节点
        neighborPtr->g_score = tentative_g_score;
        neighborPtr->f_score = neighborPtr->g_score + getHeu(neighborPtr, endPtr);
        neighborPtr->Father = currentPtr;
        neighborPtr->id = 1;
        Openset.insert(make_pair(neighborPtr->f_score, neighborPtr));
        continue;
      }
      else if(neighborPtr->id==1)
      {
        // 【修改1】：已在open set中，检查是否需要更新
        if (tentative_g_score < neighborPtr->g_score) {
          // 找到更优路径，更新节点
          neighborPtr->g_score = tentative_g_score;
          neighborPtr->f_score = neighborPtr->g_score + getHeu(neighborPtr, endPtr);
          neighborPtr->Father = currentPtr;
          
          // 必须将更新后的节点重新插入 Openset
          // 虽然旧的节点还在里面（f_score较大），但那是"懒删除"策略。
          // 当我们稍后从 Openset 拿出旧节点时，因为它的 g_score 比当前存储的大，
          // 可以通过检查忽略它，或者因为它是指针，拿出时数据已经是新的了。
          // 关键是：必须放入新的 f_score 键值对，否则该节点永远不会因 f_score 变小而被提前遍历。
          Openset.insert(make_pair(neighborPtr->f_score, neighborPtr));
        }
        // continue; // 这里不需要 continue，循环自然结束
      }
    }
  }

  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
  return false;
}


vector<Vector3d> Astarpath::getPath() {
  vector<Vector3d> path;
  vector<MappingNodePtr> front_path;
  
  // 检查 terminatePtr 是否有效
  if (terminatePtr == NULL) {
    ROS_ERROR("[A*] terminatePtr is NULL, cannot get path!");
    return path;
  }
  
  // 从终点回溯到起点
  MappingNodePtr current = terminatePtr;
  do {
    current->coord = gridIndex2coord(current->index);
    front_path.push_back(current);
    current = current->Father;
  } while (current != NULL);  // 修复：检查 current 是否为 NULL，而不是 current->Father
  
  // 将路径反转（从起点到终点）
  for (int i = front_path.size() - 1; i >= 0; i--) {
    path.push_back(front_path[i]->coord);
  }
  
  ROS_INFO("[A*] Path extracted: %zu nodes", path.size());
  if (path.size() > 0) {
    ROS_INFO("[A*] Path start: [%.2f, %.2f, %.2f], end: [%.2f, %.2f, %.2f]",
             path[0](0), path[0](1), path[0](2),
             path[path.size()-1](0), path[path.size()-1](1), path[path.size()-1](2));
  }

  return path;
}


std::vector<Vector3d> Astarpath::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {

  //init
  double dmax=0,d;
  int index=0;
  int end = path.size();
  //1.计算距离首尾连成直线最大的点，并将点集从此处分开
  for(int i=1;i<end-1;i++)
  {
    d=perpendicularDistance(path[i],path[0],path[end-1]);
    if(d>dmax)
    {
      index=i;
      dmax=d;
    }
  }
  vector<Vector3d> subPath1;
  int j = 0;
  while(j<index+1){
    subPath1.push_back(path[j]);
    j++;
  }
  vector<Vector3d> subPath2;
   while(j<int(path.size())){
    subPath2.push_back(path[j]);
    j++;
  }
  //2.拆分点集
  vector<Vector3d> recPath1;
  vector<Vector3d> recPath2;
  vector<Vector3d> resultPath;
  if(dmax>path_resolution)
  {
    recPath1=pathSimplify(subPath1,path_resolution);
    recPath2=pathSimplify(subPath2,path_resolution);
   for(int i=0;i<int(recPath1.size());i++){
    resultPath.push_back(recPath1[i]);
  }
     for(int i=0;i<int(recPath2.size());i++){
    resultPath.push_back(recPath2[i]);
  }
  }else{
    if(path.size()>1){
      resultPath.push_back(path[0]);
      resultPath.push_back(path[end-1]);
    }else{
      resultPath.push_back(path[0]);
    }
    
  }

  return resultPath;
}

double Astarpath::perpendicularDistance(const Eigen::Vector3d point_insert,const Eigen:: Vector3d point_st,const Eigen::Vector3d point_end)
{
  Vector3d line1=point_end-point_st;
  Vector3d line2=point_insert-point_st;
  return double(line2.cross(line1).norm()/line1.norm());
}

Vector3d Astarpath::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}


int Astarpath::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe

  double delta_t=resolution/1.0;//conservative advance step size;
  double t = delta_t;
  Vector3d advancePos;
  for(int i=0;i<polyCoeff.rows();i++)
  {
    while(t<time(i)){
     advancePos=getPosPoly(polyCoeff,i,t) ;
     if(isOccupied(coord2gridIndex(advancePos))){
       unsafe_segment=i;
       break;
     }   
     t+=delta_t;
    }
    if(unsafe_segment!=-1){

      break;
    }else{
      t=delta_t;
    }
  }
  return unsafe_segment;
}

void Astarpath::resetOccupy(){
      for (int i = 0; i < GRID_X_SIZE; i++)
    for (int j = 0; j < GRID_Y_SIZE; j++)
      for (int k = 0; k < GRID_Z_SIZE; k++)
        data[i * GLYZ_SIZE + j * GRID_Z_SIZE + k] = 0;
}

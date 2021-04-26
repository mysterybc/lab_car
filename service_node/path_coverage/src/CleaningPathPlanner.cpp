#include "CleaningPathPlanner.h"
#include <costmap_2d/cost_values.h>

CleaningPathPlanning::CleaningPathPlanning(costmap_2d::Costmap2DROS *costmap2d_ros)
{
    //temp solution.
    costmap2d_ros_ = costmap2d_ros;
    //costmap2d_ros_->updateMap();
    costmap2d_ = costmap2d_ros->getCostmap();
    //costmap2d_->saveMap("/home/wz/temp.pgm");


    ros::NodeHandle private_nh("~/cleaning_plan_nodehandle");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("cleaning_path", 1);
    grid_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("covered_grid",1);


    string sizeOfCellString,coveredValueStr;

    SIZE_OF_CELL = 3;
    if(private_nh.searchParam("size_of_cell",sizeOfCellString))
        private_nh.param("size_of_cell",SIZE_OF_CELL,3);

    GRID_COVERED_VALUE = 0;
    if(private_nh.searchParam("grid_covered_value",coveredValueStr))
        private_nh.param("grid_covered_value",GRID_COVERED_VALUE,0);


    int sizex = costmap2d_->getSizeInCellsX();
    int sizey = costmap2d_->getSizeInCellsY();
    // cout<<"The size of map is "<<sizex<<"  "<<sizey<<endl;
    resolution_ = costmap2d_->getResolution();

    srcMap_=Mat(sizey,sizex,CV_8U);
    for(int r = 0; r < sizey; r++){
      for(int c = 0; c < sizex; c++ ){
          srcMap_.at<uchar>(r,c) = costmap2d_->getCost(c,sizey-r-1);//??sizey-r-1 caution: costmap's origin is at left bottom ,while opencv's pic's origin is at left-top.
      }
    }

    //imshow("debugMapImage",srcMap_);
    //imshow("debugCellMatImage",cellMat_);
    //imshow("debugneuralizedMatImage",neuralizedMat_);
    //waitKey(0);
    //imwrite("debug_srcmap.jpg",srcMap_);
    

    if(!srcMap_.empty())
    {
        initialized_ = true;
        srcMap_.copyTo(mouse_select_region_Map_);
    }
    else initialized_ = false;
    
    //gzx: 我把获取机器人当前位置放到了这里，
    //melodic devel
    geometry_msgs::PoseStamped pose;   
    bool isok = costmap2d_ros_->getRobotPose(pose);   
    tf::poseStampedMsgToTF(pose,initPose_);   
    //kinetic devel
    // bool isok = costmap2d_ros_->getRobotPose(initPose_)
    if(!isok)
    {
        ROS_INFO("Failed to get robot location! Please check where goes wrong!");
        return;
    }

   //initPoint.row = initPose_.getOrigin().y()
    unsigned int mx,my;
    double wx = initPose_.getOrigin().x();
    double wy = initPose_.getOrigin().y();
    //geometry_msgs::PoseStamped current_position;
    //tf::poseStampedTFToMsg(global_pose, current_position);

    bool getmapcoor = costmap2d_->worldToMap(wx,wy,mx,my);//机器人位置转为代价地图坐标系下的位置
    if(!getmapcoor)
    {
        ROS_INFO("Failed to get robot location in map! Please check where goes wrong!");
        return;
    }
    //获取机器人位置之后，将机器人的位置，以及各区起始点的位置存入并放入mainPlanningLoop中执行
    //现在问题是如何同时运行两个mainPlanningLoop在两个不同的坐标点且划线
    cellIndex First_Point;
    First_Point.theta = 90;
    First_Point.row =(srcMap_.rows-my-1);//srcMap_.rows / SIZE_OF_CELL - 3386/SIZE_OF_CELL - 1; //此处的转换会出现问题  gzx 改过  
    First_Point.col = mx;

    pathVec_.clear();
    pathVec_.push_back(First_Point);//输入起始点
    
}

vector<geometry_msgs::PoseStamped> CleaningPathPlanning::GetPathInROS(const Mat& division_region_Map ,const Mat& R)
{
//    vector<geometry_msgs::PoseStamped> resultVec;
    //if(!pathVecInROS_.empty())pathVecInROS_.clear();
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Pose pose;
    vector<cellIndex> cellvec;
    initializeMats(division_region_Map);
    // std::cout<<"初始化图像"<<std::endl;
    cellvec = GetPathInCV();
    // cellMat_.release();
    // neuralizedMat_.release();
    freeSpaceVec_.clear();
    /*trasnsform*/
    // pathVec_[0].theta = pathVec_[0].theta *PI/180;
    // vector<cellIndex>::iterator iter;
    // int sizey = srcMap_.rows;
    // for(iter=cellvec.begin(); iter!=cellvec.end();iter++)
    // {
    //      costmap2d_->mapToWorld((*iter).col, (sizey-(*iter).row-1), pose.position.x, pose.position.y);
    //      pose.orientation.w = cos((*iter).theta/ 2);//; //(sizey-(*iter).row-1)
    //      pose.orientation.x = 0; 
    //      pose.orientation.y = 0;
    //      pose.orientation.z = sin((*iter).theta/ 2);//
    //      posestamped.header.stamp= ros::Time::now();
    //      posestamped.header.frame_id = "map";
    //      posestamped.pose = pose;
    //      //std::cout<<" x:"<<pose.position.x<<" y:"<<pose.position.y<<" theat:"<<(*iter).theta*180/PI<<std::endl;
    //      pathVecInROS_.push_back(posestamped);
    // }
    // publishPlan(pathVecInROS_);
    // cout<<"The path size is "<<pathVecInROS_.size()<<endl;
    return pathVecInROS_;
}
vector<geometry_msgs::PoseStamped> CleaningPathPlanning::GetPathInROS()
{
    vector<cellIndex> cellvec;
    initializeMats(division_region_Map_);
    cellvec = GetPathInCV();
    /*trasnsform*/
    // cellvec[0].theta = cellvec[0].theta *PI/180;
    // vector<cellIndex>::iterator iter;
    // int sizey = srcMap_.rows;
    // for(iter=cellvec.begin(); iter!=cellvec.end();iter++)
    // {
    //      costmap2d_->mapToWorld((*iter).col, (sizey-(*iter).row-1), pose.position.x, pose.position.y);
    //      pose.orientation.w = cos((*iter).theta/ 2);//; //(sizey-(*iter).row-1)
    //      pose.orientation.x = 0; 
    //      pose.orientation.y = 0;
    //      pose.orientation.z = sin((*iter).theta/ 2);//
    //      posestamped.header.stamp= ros::Time::now();
    //      posestamped.header.frame_id = "map";
    //      posestamped.pose = pose;
    //      std::cout<<" x:"<<pose.position.x<<" y:"<<pose.position.y<<" theat:"<<(*iter).theta*180/PI<<std::endl;
    //      pathVecInROS_.push_back(posestamped);
    // }
    // publishPlan(pathVecInROS_);
    // cout<<"The path size is "<<pathVecInROS_.size()<<endl;
    // return pathVecInROS_;
}

vector<geometry_msgs::PoseStamped> CleaningPathPlanning::trasnsformtorospath()
{
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Pose pose;
    pathVec_[0].theta = pathVec_[0].theta *PI/180;
    vector<cellIndex>::iterator iter;
    int sizey = srcMap_.rows;
    for(iter=(++pathVec_.begin()); iter!=pathVec_.end();iter++)
    {
         costmap2d_->mapToWorld((*iter).col, (sizey-(*iter).row-1), pose.position.x, pose.position.y);
         pose.orientation.w = cos((*iter).theta/ 2);//; //(sizey-(*iter).row-1)
         pose.orientation.x = 0; 
         pose.orientation.y = 0;
         pose.orientation.z = sin((*iter).theta/ 2);//
         posestamped.header.stamp= ros::Time::now();
         posestamped.header.frame_id = "map";
         posestamped.pose = pose;
        //  std::cout<<" x:"<<pose.position.x<<" y:"<<pose.position.y<<" theat:"<<(*iter).theta*180/PI<<std::endl;
         pathVecInROS_.push_back(posestamped);
    }
    publishPlan(pathVecInROS_);
    // cout<<"The path size is "<<pathVecInROS_.size()<<endl;
    return pathVecInROS_;

}

bool CleaningPathPlanning::initializeMats(const Mat& division_region_Map)
{
    //initialize the member variables.
    if(srcMap_.empty())return false;
    getCellMatAndFreeSpace(division_region_Map,cellMat_,freeSpaceVec_);//gzx修改部分：srcMap_  再加一个函数
    //getCellMatAndFreeSpace(division_region_Map_,cellMat_,freeSpaceVec_);
    neuralizedMat_ = Mat(cellMat_.rows,cellMat_.cols,CV_32F);
    initializeNeuralMat(cellMat_,neuralizedMat_);
    return true;
}

void CleaningPathPlanning::getCellMatAndFreeSpace(const Mat& srcImg, Mat &cellMat,vector<cellIndex> &freeSpaceVec)
{
    cellMat = Mat(srcImg.rows / SIZE_OF_CELL,srcImg.cols / SIZE_OF_CELL, srcImg.type());

    freeSpaceVec.clear();
    bool isFree = true;
    int r = 0,c = 0, i = 0,j = 0;
    for (r = 0;r < cellMat.rows; r++ )
    {
        for(c = 0 ; c<cellMat.cols; c++)
        {
            isFree = true;
            for(i = 0; i<SIZE_OF_CELL; i++)
            {
                for(j = 0; j < SIZE_OF_CELL; j++)
                {
                    if(srcImg.at<uchar>(r*SIZE_OF_CELL+i,c*SIZE_OF_CELL+j) != costmap_2d::FREE_SPACE)
                    {
                        isFree=false;
                        i = SIZE_OF_CELL;
                        break;
                    }
                }
            }
            if(isFree)
            {
                cellIndex ci;
                ci.row = r;
                ci.col = c;
                ci.theta = 0;
                freeSpaceVec.push_back(ci);
                cellMat.at<uchar>(r,c) = costmap_2d::FREE_SPACE;//255
            }
            else{cellMat.at<uchar>(r,c) = costmap_2d::LETHAL_OBSTACLE;}//253
        }
    }
    // cout<<"freespace size:"<< freeSpaceVec.size()<<endl;
    // imwrite("/home/guozixuan/cellMat.jpg",cellMat);
    return;
}

void CleaningPathPlanning::initializeNeuralMat(Mat cellMat, Mat neuralizedMat)
{
    int i = 0,j = 0;
    for(i = 0; i < neuralizedMat.rows; i++)
    {
        for(j = 0;j< neuralizedMat.cols; j++ )
        {
            if(cellMat.at<uchar>(i,j) == costmap_2d::LETHAL_OBSTACLE) neuralizedMat.at<float>(i,j) = -100000.0;//这里是不是可以
            else neuralizedMat.at<float>(i,j) = 50.0 / j;   //这里原来是1.0 
        }
    }
    return;
}


void CleaningPathPlanning::SetCoveredGrid(double wx, double wy)
{
    unsigned int mx,my,index;
    bool isok = costmap2d_->worldToMap(wx,wy,mx,my);
    if(!isok)
    {
        return;
    }

    for(int dx = -SIZE_OF_CELL/2; dx < SIZE_OF_CELL/2+1; dx++)
     {
        for(int dy =-SIZE_OF_CELL/2; dy<SIZE_OF_CELL/2+1; dy++)
        {
            index = costmap2d_->getIndex(mx+dx,my+dy);
            covered_path_grid_.data[index] = GRID_COVERED_VALUE;
        }
    }
}

void CleaningPathPlanning::PublishGrid()
{
    if(!initialized_)initializeCoveredGrid();
    grid_pub_.publish(covered_path_grid_);
}

vector<cellIndex> CleaningPathPlanning::GetPathInCV()
{

    int times = init_point_.size();

    mainPlanningLoop(init_point_[0].x,init_point_[0].y);

    for(int i =0;i<temp_pathVec_.size();i++)
    {
       pathVec_.push_back(temp_pathVec_[i]); //转入pathVec_
    }
    // std::cout<<"pathVec_.size()"<<pathVec_.size()<<std::endl;
    
    return this->temp_pathVec_;
}

void CleaningPathPlanning::PublishCoveragePath(){
  publishPlan(this->pathVecInROS_);
}

void CleaningPathPlanning::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
{
    if (!initialized_) {
        ROS_ERROR(
           "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }
    plan_pub_.publish(gui_path);
}

bool CleaningPathPlanning::cellContainsPoint(Point2i pt, cellIndex cell)
{
    return pt.y>cell.row*SIZE_OF_CELL && pt.y<(cell.row+1)*SIZE_OF_CELL
            && pt.x>cell.col*SIZE_OF_CELL && pt.x<(cell.col+1)*SIZE_OF_CELL;
}

void CleaningPathPlanning::writeResult(Mat resultmat,vector<cellIndex> pathVec)
{
    int i = 0,j = 0;
    Point initpoint = Point(pathVec[0].col,pathVec[0].row);
    for(i = 1; i<pathVec.size();i++)
    {
        Point cupoint = Point(pathVec[i].col,pathVec[i].row);
        fov_middlepoint_path_.push_back(cupoint);
        if(sqrt((initpoint.x-cupoint.x)*(initpoint.x-cupoint.x)+(initpoint.y-cupoint.y)*(initpoint.y-cupoint.y))>2)
        {
            line(resultmat,initpoint,cupoint,Scalar(0,255,0),0.3,8);
        }
        else line(resultmat,initpoint,cupoint,Scalar(0,0,255),0.5);
        initpoint = cupoint;
        cout << "The point of step "<<i<<" is: "<<pathVec[i].row<<" "<<pathVec[i].col<<endl;
    }

    // imwrite("/home/guozixuan/reaultMat.jpg",resultmat);
}

void CleaningPathPlanning::writeResult(Mat resultmat,vector<cv::Point2f> pathVec)
{
    int i = 0,j = 0;
    Point initpoint = Point(round(pathVec[0].x),round(pathVec[0].y));
    for(i = 1; i<pathVec.size();i++)
    {
        Point cupoint = Point(round(pathVec[i].x),round(pathVec[i].y));
        line(resultmat,initpoint,cupoint,255,3);
        initpoint = cupoint;
        //std::cout<<"X: "<<cupoint.x<<","<<"Y:"<<cupoint<<std::endl;

    }
    //by bc
    // namedWindow("resultMat",CV_WINDOW_NORMAL);
    // imshow("resultMat",resultmat);
    // waitKey(0);
    // cv::destroyWindow("resultMat");
}

void CleaningPathPlanning::mainPlanningLoop(int point_img_x , int point_img_y)
{
    temp_pathVec_.clear();
    cellIndex initPoint,nextPoint, currentPoint;
//    initPoint.row = cellMat_.rows/2; //initPoint to be made interface.
//    initPoint.col = cellMat_.cols/2;
    initPoint.theta = 90;

    initPoint.row = point_img_y/SIZE_OF_CELL;//srcMap_.rows / SIZE_OF_CELL - 3386/SIZE_OF_CELL - 1; //此处的转换会出现问题  gzx 改过  
    initPoint.col = point_img_x/SIZE_OF_CELL;


    currentPoint = initPoint;
    //pathVec_.clear();
    temp_pathVec_.push_back(initPoint);

    float initTheta = initPoint.theta; //initial orientation
    const float c_0 = 50;//后面要用的参数0.001
    float e = 0.0, v = 0.0, deltaTheta = 0.0, lasttheta = initTheta, PI = 3.14159;
    vector<float> thetaVec = {0,45,90,135,180,225,270,315};

    for(int loop=0;loop<9000;loop++)
    {
        //erase current point from free space first.
        vector<cellIndex>::iterator it;
	/*
	for(it=freeSpaceVec_.begin();it!=freeSpaceVec_.end();)
        {
            if((*it).row==nextPoint.row && (*it).col==nextPoint.col)
            {it = freeSpaceVec_.erase(it);continue;}
            it++;
        }

        */
        //compute neiborhood's activities
        //hjr注：目前我认为这里进行的是有关方向上的抉择。
        int maxIndex = 0;//目前尚不清楚这两个参数最后是干啥的。
        float max_v = -300;
        neuralizedMat_.at<float>(currentPoint.row ,currentPoint.col) = -250.0;//把当前点赋值为-250
	    lasttheta = currentPoint.theta;
        for(int id = 0; id < 8; id++)
        {
            deltaTheta = max(thetaVec[id],lasttheta)-min(thetaVec[id],lasttheta);
            if(deltaTheta>180) deltaTheta=360-deltaTheta;
            e = 1 - abs(deltaTheta) / 180;//角度参数？
            switch (id)
            {
                case 0:
                    if(currentPoint.col==neuralizedMat_.cols-1){v=-100000;break;}//处于边界？
                    v = neuralizedMat_.at<float>(currentPoint.row ,currentPoint.col+1) + c_0 * e;
		    
                    break;
                case 1:
                if(currentPoint.col==neuralizedMat_.cols-1 || currentPoint.row == 0){v=-100000;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row-1 ,currentPoint.col+1) + c_0 * e-200;//-200

                    break;
                case 2:
                if(currentPoint.row == 0){v=-100000;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row-1 ,currentPoint.col) + c_0 * e;

                    break;
                case 3:
                if(currentPoint.col== 0 || currentPoint.row == 0){v=-100000;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row-1 ,currentPoint.col-1) + c_0 * e-200;//-200

                    break;
                case 4:
                if(currentPoint.col== 0){v=-100000;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row ,currentPoint.col-1) + c_0 * e;

                    break;
                case 5:
                if(currentPoint.col== 0 || currentPoint.row == neuralizedMat_.rows-1){v=-100000;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row+1 ,currentPoint.col-1) + c_0 * e-200;//-200

                    break;
                case 6:
                if(currentPoint.row == neuralizedMat_.rows-1){v=-100000;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row+1 ,currentPoint.col) + c_0 * e;

                    break;
                case 7:
                if(currentPoint.col==neuralizedMat_.cols-1 || currentPoint.row == neuralizedMat_.rows-1){v=-100000;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row+1 ,currentPoint.col+1) + c_0 * e-200;//-200

                    break;
                default:
                    break;
            }
            if(v > max_v)
            {
                max_v = v;
                maxIndex=id;
            }
            
            if(v== max_v&&id>maxIndex)
            {
                max_v = v;
                maxIndex = id;
            }

        }

        //什么情况下会出现max_v<=0，也就是所有的方向v都小于0
        if(max_v <= 0)//接下来应该是在处理距离上的关系。找到最近的点跳过去？
        {
            float dist = 0.0, min_dist = 100000000;
            //vector<cellIndex>::iterator min_iter;
            int ii=0, min_index=-1;
            for(it=freeSpaceVec_.begin();it!=freeSpaceVec_.end();it++)
            {
                if(neuralizedMat_.at<float>((*it).row,(*it).col) > 0)
                { 
                    if(Boundingjudge((*it).row,(*it).col))//周围是否存在-250的点
                    {
                        dist = sqrt((currentPoint.row-(*it).row)*(currentPoint.row-(*it).row)+(currentPoint.col-(*it).col)*(currentPoint.col-(*it).col));
                            if(dist < min_dist)
                                {
                                    min_dist = dist;
                                    min_index = ii;
                                }
                    }
                }
                ii++;
            }
            //if(min_dist==0 || min_index == -1)
            //{break;}
            //else
            if(min_index!=-1&&min_dist!=100000000)
             {
                // cout << "next point index: "<<min_index<< endl;
                // cout << "distance: "<<min_dist << endl;
                nextPoint = freeSpaceVec_[min_index];
                currentPoint = nextPoint;
                temp_pathVec_.push_back(nextPoint);

                continue; 
             }
             else //产生了自锁现象
                {
                    ROS_INFO("The program has been dead because of the self-locking");
                    // ROS_ERROR("The program has gone through %d steps", pathVec_.size());
                    break;
                }
        }

        //next point.
        switch (maxIndex)
        {
        case 0:
            nextPoint.row = currentPoint.row;
            nextPoint.col = currentPoint.col+1;
            break;
        case 1:
            nextPoint.row = currentPoint.row-1;
            nextPoint.col = currentPoint.col+1;
            break;
        case 2:
            nextPoint.row = currentPoint.row-1;
            nextPoint.col = currentPoint.col;
            break;
        case 3:
            nextPoint.row = currentPoint.row-1;
            nextPoint.col = currentPoint.col-1;
            break;
        case 4:
            nextPoint.row = currentPoint.row;
            nextPoint.col = currentPoint.col-1;
            break;
        case 5:
            nextPoint.row = currentPoint.row+1;
            nextPoint.col = currentPoint.col-1;
            break;
        case 6:
            nextPoint.row = currentPoint.row+1;
            nextPoint.col = currentPoint.col;
            break;
        case 7:
            nextPoint.row = currentPoint.row+1;
            nextPoint.col = currentPoint.col+1;
            break;
        default:
            break;
        }
        nextPoint.theta = thetaVec[maxIndex];
        currentPoint = nextPoint;
        temp_pathVec_.push_back(nextPoint);
    }
    fov_middlepoint_path_.clear();
     //数据转换
    for(int i = 0;i < temp_pathVec_.size();i++)
    {
        cv::Point2f point;
        point.y = temp_pathVec_[i].row * SIZE_OF_CELL + SIZE_OF_CELL/2;
        point.x = temp_pathVec_[i].col * SIZE_OF_CELL + SIZE_OF_CELL/2;
        //std::cout<<"原始图像存入的"<<point<<std::endl;
        fov_middlepoint_path_.push_back(point);
    }
    cv::Mat R_inv;
	cv::invertAffineTransform(R_, R_inv);
    // std::cout<<"clr.R_"<<R_<<std::endl;
    std::vector<cv::Point2f> fov_middlepoint_path_transformed;
    fov_middlepoint_path_transformed.clear();
	cv::transform(fov_middlepoint_path_, fov_middlepoint_path_transformed, R_inv);
    std::vector<geometry_msgs::Pose2D> pose_path;
    pose_path.clear();
    transformPointPathToPosePath(fov_middlepoint_path_transformed,pose_path);
    for(int i = 0;i < fov_middlepoint_path_transformed.size();i++)
    {
        //这个地方对point进行了四舍五入
        cv::Point point = Point(round(fov_middlepoint_path_transformed[i].x),round(fov_middlepoint_path_transformed[i].y));
        temp_pathVec_[i].row = point.y ;
        temp_pathVec_[i].col = point.x ;
        temp_pathVec_[i].theta = pose_path[i].theta;
        
    }
    Mat resultMat = srcMap_.clone();
    writeResult(resultMat,fov_middlepoint_path_transformed);
    
    //writeResult(resultMat,temp_pathVec_);
}

double CleaningPathPlanning::distance(Point2i pta, Point2i ptb)
{
    return sqrt((pta.x-ptb.x)*(pta.x-ptb.x)+(pta.y-ptb.y)*(pta.y-ptb.y));
}

bool CleaningPathPlanning::findElement(vector<Point2i> pointsVec, Point2i pt, int &index)
{
    for(int i = 0;i<pointsVec.size();i++)
    {
        if(pointsVec[i].x == pt.x && pointsVec[i].y == pt.y)
           { index = i;
        return true;}
    }
    index = -1;
    return false;
}



bool CleaningPathPlanning::initializeCoveredGrid()
{
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap2d_->getMutex()));
    double resolution = costmap2d_->getResolution();

    covered_path_grid_.header.frame_id = "map";
    covered_path_grid_.header.stamp = ros::Time::now();
    covered_path_grid_.info.resolution = resolution;

    covered_path_grid_.info.width = costmap2d_->getSizeInCellsX();
    covered_path_grid_.info.height = costmap2d_->getSizeInCellsY();

    double wx, wy;
    costmap2d_->mapToWorld(0, 0, wx, wy);
    covered_path_grid_.info.origin.position.x = wx - resolution / 2;
    covered_path_grid_.info.origin.position.y = wy - resolution / 2;
    covered_path_grid_.info.origin.position.z = 0.0;
    covered_path_grid_.info.origin.orientation.w = 1.0;

    covered_path_grid_.data.resize(covered_path_grid_.info.width * covered_path_grid_.info.height);

    unsigned char* data = costmap2d_->getCharMap();
    for (unsigned int i = 0; i < covered_path_grid_.data.size(); i++)
    {
        /*if(data[i]==costmap_2d::FREE_SPACE)
            covered_path_grid_.data[i] = costmap_2d::FREE_SPACE;
        else
            covered_path_grid_.data[i] = 0;*/
        covered_path_grid_.data[i] = data[i];
    }
    return true;
}

//hjr---------------------------------------------------------
bool CleaningPathPlanning::Boundingjudge(int a,int b)
{
  int num = 0;
  for(int i = -1;i <= 1;i++)
  {
    for(int m = -1;m <= 1;m++)
    {
      if(i == 0&&m == 0) 
      {
	    continue;
      }
      if(neuralizedMat_.at<float>((a+i),(b+m))==-250.0)
      num++;
    }
  }
  if(num!=0)
    return true;
  else
    return false;
}

bool CleaningPathPlanning::outcloselist(int a, int b)
{
vector<Astar>::iterator it;
 for(it=closelist.begin();it!=closelist.end();)
       {
            if((*it).row==a && (*it).col==b)
            {
	     return false;
	      break;
	    }
           it++;
       }
       return true;
}



bool CleaningPathPlanning::outopenlist(int a, int b)
{
    vector<Astar>::iterator it;
    for(it=openlist.begin();it!=openlist.end();)
       {
            if((*it).row==a && (*it).col==b)
            {
	      G_compare = (*it).G;
	     return false;
	      break;
	    }
           it++;
       }
       return true;
}


void CleaningPathPlanning::Astar_find_path(int a, int b, int c, int d)
{
   int F=0;
   int G=0;
   int H=0;
   
   int min_F =100000;
   bool SW = false;
  
   Astar initial;
   Astar temp;
   Astar current;
   Astar next;
   initial.row = a;
   initial.col  = b;
   initial.father_row=a;
   initial.father_col=b;
   initial.F = 0;
   initial.G = 0;
   initial.H = 0;
   //----------------------------------------------------------
   openlist.push_back(initial);//加入初始点
   //------------------------------------------------
   
   vector<Astar>::iterator ite;
 for(ite=openlist.begin();ite!=openlist.end();)
 {
   if((*ite).F<=min_F)
   {
     min_F = (*ite).F;
     current = (*ite);
   }
   ite++;
 }
 
 
 
 //--------------------------------------------------
vector<Astar>::iterator it;

 for(it=openlist.begin();it!=openlist.end();)
       {
            if((*it).row==current.row && (*it).col==current.col)
            {
	      closelist.push_back((*it));
	      openlist.erase(it);
	      break;
	    }
           it++;
       }

 //-------------------------------------------------------
   for(int i=-1;i<=1;i++)
  {
    for(int m=-1;m<=1;m++)
    {
      if(i==0&&m==0) 
      {
	continue;
      }
     if(Astarmap.at<float>((current.row+i),(current.col+m))!=-100000.0||outcloselist((current.row+i),(current.col+m)))
     {
       if(!outopenlist((current.row+i),(current.col+m)))
       {
	 if(i==0||m==0)
	 {
	   if((current.G+10)<G_compare)//G值更小，説明更好.
	   {
              temp.father_row = current.row;
              temp.father_col =current.col;
	      temp.G = current.G+10;
	   }
	 }
	 else
	 {
	     if((current.G+14)<G_compare)//G值更小，説明更好.
	   {
              temp.father_row = current.row;
              temp.father_col =current.col;
	      temp.G = current.G+14;
	   }
	 }
	 
       } 
       temp.row = current.row+i;
       temp.col =current.col+m;
       temp.father_row = current.row;
       temp.father_col =current.col;
       
       
       if(i==0||m==0)
       {
	 temp.G = 10+current.G;
       }
       else
       {
	 temp.G = 14+current.G;
       }
        
        temp.H = (abs(c-temp.row) + abs(d-temp.col))*10;
	temp.F = temp.G + temp.H;
       
       openlist.push_back(temp);
     }
    }
  }
  
 
//--------------------------------------------------
// vector<Astar>::iterator it;
// 
//  for(it=openlist.begin();it!=openlist.end();)
//        {
//             if((*it).row==current.row && (*it).col==current.col)
//             {
// 	      closelist.push_back((*it));
// 	      openlist.erase(it);
// 	      break;
// 	    }
//            it++;
//        }

 //------------------------------------------------------------找到需要的点了吗？
 for(it=openlist.begin();it!=openlist.end();)
       {
            if((*it).row==c && (*it).col==d)
            {
	      SW = false;
	      break;
	    }
           it++;
       }
 
}
void CleaningPathPlanning::transformPointPathToPosePath(const std::vector<cv::Point2f>& point_path, std::vector<geometry_msgs::Pose2D>& pose_path)
{
	// create poses with an angle
	for(size_t point_index = 0; point_index < point_path.size(); ++point_index)
	{
		// get the vector from the previous to the current point
		const cv::Point2f& current_point = point_path[point_index];

		// add the next navigation goal to the path
		geometry_msgs::Pose2D current_pose;
		current_pose.x = current_point.x;
		current_pose.y = current_point.y;
		current_pose.theta = 0.;
		cv::Point2f vector(0,0);
		if (point_index > 0)
		{
			// compute the direction as the line from the previous point to the current point
			vector = current_point - point_path[point_index-1];
		}
		else if (point_path.size() >= 2)
		{
			// for the first point take the direction between first and second point
			vector = point_path[point_index+1] - current_point;
		}
		// only sample different points
		if (vector.x!=0 || vector.y!=0)
		{
			current_pose.theta = std::atan2(-vector.y, vector.x);//这里改成-y是为了让图片坐标系与栅格地图坐标系对其。
			pose_path.push_back(current_pose);
		}
	}
}
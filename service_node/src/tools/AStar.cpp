#include "AStar.hpp"
#include <algorithm>
#include <iostream>
#include <array>
#include <fstream>


using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

bool AStar::Vec2i::is_equal(const Vec2i& coordinates_1,const Vec2i& coordinates_2)
{
    return (coordinates_1.x == coordinates_2.x && coordinates_1.y == coordinates_2.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.insert(new Node(source_));
    //判断目标点是否在障碍物上
    Vec2i newCoordinates(target_);
    if(detectCollision(newCoordinates)){
        CoordinateList path;
        return path;
    }

    //寻找路径
    while (!openSet.empty()) {
        current = *openSet.begin();
        //寻找cost最小的点
        for (auto node : openSet) {
            if (node->getScore() <= current->getScore()) {
                current = node;
            }
        }
        //结束条件
        if (current->coordinates.is_equal(target_,current->coordinates)) {
            break;
        }
        //将current节点从openlist移动到closedset
        closedSet.insert(current);
        openSet.erase(std::find(openSet.begin(), openSet.end(), current));

        //遍历current周边的节点，数量和是否允许对角线移动有关
        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }
            //计算新节点已经花费的cost
            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            //判断新节点是否在openlist中
            Node *successor = findNodeOnList(openSet, newCoordinates);
            //如果不在的话，加入到openlist中，并计算H、G
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                //std::cout << "将新的点加入openlist,x is :" << newCoordinates.x << " ,y is :" << newCoordinates.y << std::endl;
                openSet.insert(successor);
            }
            //如果在openlist中，并且已经走过的花费比较大的话，重置该节点的花费
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);
    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates.is_equal(coordinates_,node->coordinates)) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(100 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

void AStar::Generator::LoadMap(std::string map_path, AStar::Generator &astar_planner){
	astar_planner.setHeuristic(AStar::Heuristic::euclidean);
	astar_planner.setDiagonalMovement(true);
	cv::Mat img_raw, img,temp_img;
	img_raw = cv::imread(map_path,cv::IMREAD_GRAYSCALE);
	threshold(img_raw, img, 100, 255, CV_THRESH_BINARY);

    //膨胀障碍物,使用9*9的mask
    {
        temp_img = img.clone();
        int i,j,rgb;
        for (size_t r = 0; r < img.rows; r++){
		    for (size_t c = 0; c < img.cols; c++){
                rgb = (int)img.at<uchar>(r , c);
                if(rgb == 0){
                    for(int i = r - 16; i <= r + 16 ; i++){
                        for(int j = c - 16; j <= c + 16 ; j++){
                            if( i < 0 || i > img.rows || j < 0 || j > img.cols)
                                continue;
                            temp_img.at<uchar>(i , j) = 0;
                        }
                    }
                }
            }
        }
    }


	astar_planner.setWorldSize({ temp_img.cols, temp_img.rows });
    std::array<AStar::Vec2i,9> mask = {{{-1,-1},{-1,0},{-1,1},
                                        { 0,-1},{ 0,0},{ 0,1},
                                        { 1,-1},{ 1,0},{ 1,1}}};
    int bgr;
    bool wall_flag{false};
	for (size_t r = 0; r < temp_img.rows; r++){
		for (size_t c = 0; c < temp_img.cols; c++){
            wall_flag = false;
            for(int i = 0; i < mask.size() - 1; i++){
                if(r + mask.at(i).x < 0 || r + mask.at(i).x > temp_img.rows || c + mask.at(i).y <0 || c + mask.at(i).y >temp_img.cols)
                    continue;
                bgr = (int)temp_img.at<uchar>(r + mask.at(i).x, c + mask.at(i).y);
                if(bgr == 0)
                    wall_flag = true;    
            }
			if (wall_flag == true)
            {
                astar_planner.addCollision({ c, temp_img.rows-r });
            }
				
		}
	}





}
import logging
import sys
import re


logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',filemode='a')


def main():
    if len(sys.argv) != 5:
        logging.info('please input args: car_path, road_path, cross_path, answerPath')
        exit(1)

    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    answer_path = sys.argv[4]

    logging.info("car_path is %s" % (car_path))
    logging.info("road_path is %s" % (road_path))
    logging.info("cross_path is %s" % (cross_path))
    logging.info("answer_path is %s" % (answer_path))

    return car_path,road_path,cross_path,answer_path

# to read input file

def loadDataSet(filename):
    dict = {}
    fr = open(filename)
    information = []
    for line in fr.readlines():
        if line[0] != '#':
            line.strip().split()
            line_list = re.findall(r'[(](.*?)[)]', line)
            save_list = line_list[0].split(',')
            save_list = [int(i) for i in save_list]
            information.append(save_list)

    for i in range(len(information)):
        key = information[i][0]
        value = information[i][1:]
        dict.update({key:value})
    return dict

# process
class Point:
    """
    表示一个点   使用index 表示cross的编号，也就是将每一个cross当作一个Point
    """
    def __init__(self, index):
        self.index = index
    def __eq__(self, other):
        if self.index == other.index:
            return True
        return False
    def __str__(self):
        return str(self.index)

class Node_Attributes:
    def __init__(self,point,EndPoint,g_Value = 0):
        self.EndPoint = EndPoint
        self.point = point    # 自己这个节点本身的信息

        self.father = None    # 父节点
        self.road = None      # 本个节点和父节点之间的road

        self.g_Value = g_Value       # 起点到这里的代价
        '''a = abs(math.ceil(EndPoint.index/dimen))
        b = abs(math.ceil(point.index/dimen))
        c = abs(EndPoint.index - point.index)%8
        self.h_Value = abs(b-a)*10+ abs(c)*10'''
        self.h_Value = abs(EndPoint.index - point.index)*10
        # 这里的h值先用网格表示，后需更改

class A_star:
    def __init__(self, startPoint , endPoint , passTag = 0):
        # self.dimen = dimen       # cross_dict 的维度
        self.open_list = []
        self.close_list = []
        # self.map = map         # cross_dict 的所有索引
        self.startPoint = startPoint
        self.endPoint = endPoint
        self.passTag = passTag

    def getMinNode(self):
        """
        获得openlist中F值最小的节点
        :return: Node
        """
        currentNode = self.open_list[0]
        for node in self.open_list:
            if node.g_Value + node.h_Value < currentNode.g_Value + currentNode.h_Value:
                currentNode = node
        return currentNode

    def pointInCloseList(self, point):        # 检查这个Node是否已经在 closelist 中了
        for node in self.close_list:
            if node.point == point:
                return True
        return False

    def pointInOpenList(self, point):        #检查这个Node是否已经在 openlist 中了
        for node in self.open_list:
            if node.point == point:
                return node
        return None

    def endPointInCloseList(self):           # 检查终点是否已经在closedList中了
        for node in self.open_list:
            if node.point == self.endPoint:
                return node
        return None

    def searchNear(self, minF ):
        """
        搜索节点周围的点
        :param minF:F值最小的节点，也就是cross的索引
        """
        index_cross = minF.point.index
        for road_connect in cross_dict[index_cross]:
            if road_connect == -1:      # 如果road_connect = -1 则道路不存在
                continue
            if road_dict[road_connect][5] == 0 and road_dict[road_connect][3] != index_cross:  # 如果是单向而且起点不是index_cross
                continue

            if road_dict[road_connect][4] == index_cross:               # 有问题 ：一条路的两端为【3】和【4】
                currentPoint = Point(road_dict[road_connect][3])        # 已修改，加入判断，现在能够实现完整的临近点搜索
            else:
                currentPoint = Point(road_dict[road_connect][4])

            if self.pointInCloseList(currentPoint):
                continue
            step = road_dict[road_connect][0]
            currentNode = self.pointInOpenList(currentPoint) #检查currentPoint在不在openlist中
            # 如果不再openList中，就把它加入openlist
            if not currentNode:
                currentNode = Node_Attributes(currentPoint , self.endPoint , g_Value = minF.g_Value + step )
                currentNode.father = minF
                self.open_list.append(currentNode)
                continue
            # 如果在openList中，判断minF到当前点的G是否更小
            if minF.g_Value + step < currentNode.g_Value:  # 如果更小，就重新计算g值，并且改变father
                currentNode.g_Value = minF.g_Value + step
                currentNode.father = minF

    def start(self):
        """
        开始寻路
        :return: None或Point列表（路径）
        """
        # 1.将起点放入开启列表
        startNode = Node_Attributes(self.startPoint, self.endPoint)
        self.open_list.append(startNode)
        # 2.主循环逻辑
        while True:
            # 找到F值最小的点
            minF = self.getMinNode()
            # 把这个点加入closeList中，并且在openList中删除它
            self.close_list.append(minF)
            self.open_list.remove(minF)
            # 判断这个节点的上下左右节点
            self.searchNear(minF)
            # 判断是否终止
            point = self.endPointInCloseList()
            if point:  # 如果终点在关闭表中，就返回结果
                # print("关闭表中")
                cPoint = point
                pathList = []
                roadList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point)
                        aPoint = cPoint        #  添加的
                        cPoint = cPoint.father

                        for i in cross_dict[aPoint.point.index]:
                            if i == -1:  # 如果road_connect = -1 则道路不存在
                                continue
                            if (road_dict[i][4] == aPoint.point.index) and (road_dict[i][3] == cPoint.point.index):
                                roadList.append(i)
                            if (road_dict[i][3] == aPoint.point.index) and (road_dict[i][4] == cPoint.point.index):
                                roadList.append(i)
                            else:
                                continue
                    else:
                        # print(pathList)
                        # print(list(reversed(pathList)))
                        # print(pathList.reverse())

                        return list(reversed(pathList)),list(reversed(roadList))
            if len(self.open_list) == 0:
                return None

# to write output file

def writeAnswer(filename,answer):

    answer_output = answer[1:-1]
    with open(filename,'a') as fr:
        #fr.write('(' + answer + ')'+'\n',encoding = "UTF-8")
        return fr.write('('+ answer_output + ')'+'\n')

if __name__ == '__main__':
    
    car_path,road_path,cross_path,answer_path = main()   # 获取系统参数
    car_dict = {}
    road_dict = {}
    cross_dict = {}
    key_list = []
    answer_dict = {}
    cross_dict = loadDataSet(cross_path)
    road_dict = loadDataSet(road_path)
    car_dict = loadDataSet(car_path)

    for key_car in car_dict:
        answer_list = []
        carAndTime = []
        carAndTime.append(key_car)
        carAndTime.append(car_dict[key_car][3]*80)

        answer_list.append(car_dict[key_car][0])
        aStar = A_star(Point(car_dict[key_car][0]),Point(car_dict[key_car][1]))
        # 开始寻路
        pathList , roadList = aStar.start()
        '''
        for i in pathList:
            answer_list.append(i.index)
        #print(answer_list)    # 输出寻路轨迹的部分'''
        #print(carAndTime + roadList)
        writeAnswer(answer_path,str(carAndTime + roadList))
        # answer_dict.update({key_car:answer_list})
    #print(answer_dict)
    #print(roadList)


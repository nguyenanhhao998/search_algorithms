import numpy as np
from numpy.random import choice
import matplotlib.pyplot as plt
import math
from sys import argv
#Định nghĩa đồ thị
class _Graph:
    #Khởi tạo bản đồ: 
    #size: kích thước (width, height), 
    #start: tọa độ điểm bắt đâu, goal: tọa độ điểm đích, 
    #polygons: list các đa giác chắn đường, mỗi đa giác là list tọa độ các điểm theo chiều kim đồng hồ
    def __init__(self,size,start,goal,polygons,pickup=[]):
        self.size=size
        self.start=start
        self.goal=goal
        self.polygons=polygons
        self.pickup=pickup
    def setPolygons(self, polygons):
        self.polygons = polygons

    #show bản đồ và đường đi tìm được với result là danh sách cách điểm trên đường đi (tĩnh)
    def draw(self, result):
        frame=plt.Polygon([(0,0),(0,self.size[1]),(self.size[0],self.size[1]),(self.size[0],0)],fill=None,edgecolor='r',lw=5)
        plt.gca().add_patch(frame)
        plt.scatter(self.start[0],self.start[1],s=100,c="red")
        plt.scatter(self.goal[0],self.goal[1],s=100,c="red")
        plt.text(self.start[0]+0.1, self.start[1]+0.1, "S", fontsize=15)
        plt.text(self.goal[0]+0.1, self.goal[1]+0.1, "G", fontsize=15)
        for i in range(len(self.pickup)):
            plt.scatter([self.pickup[i][0]],[self.pickup[i][1]],s=70,c="green")
        for polygon in self.polygons:
            shape=plt.Polygon(polygon)
            plt.gca().add_patch(shape)
        plt.plot([v[0] for v in result], [v[1] for v in result])
        plt.xlim(0,self.size[0])
        plt.ylim(0,self.size[1])
        plt.gca().set_xticks(np.arange(0,self.size[0]+1,1))
        plt.gca().set_yticks(np.arange(0,self.size[1]+1,1))
        plt.gca().grid(True)
        #plt.axis('scaled')
        
        plt.show()

    #Show bản đồ (động)
    def drawAnimate(self, result, listChangedPolygons = []):
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)

        plt.xlim(0,self.size[0])
        plt.ylim(0,self.size[1])
        plt.gca().set_xticks(np.arange(0,self.size[0]+1,1))
        plt.gca().set_yticks(np.arange(0,self.size[1]+1,1))
        plt.gca().grid(True)

        frame=plt.Polygon([(0,0),(0,self.size[1]),(self.size[0],self.size[1]),(self.size[0],0)],fill=None,edgecolor='r',lw=5)
        plt.gca().add_patch(frame)
        plt.scatter(self.start[0],self.start[1],s=100,c="red")
        plt.scatter(self.goal[0],self.goal[1],s=100,c="red")
        for i in range(len(self.pickup)):
            plt.scatter([self.pickup[i][0]],[self.pickup[i][1]],s=70,c="green")
        plt.text(self.start[0]+0.1, self.start[1]+0.1, "S", fontsize=15)
        plt.text(self.goal[0]+0.1, self.goal[1]+0.1, "G", fontsize=15)

        shape=[]
        for polygon in self.polygons:
            shape.append(plt.Polygon(polygon))
            plt.gca().add_patch(shape[len(shape)-1])
        fig.canvas.draw()
        plt.pause(1)
        if (listChangedPolygons == []):
            for i in range(1,len(result)):
                plt.scatter(result[i][0],result[i][1],s=30)

                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.1)
        else:
            for i in range(1,len(result)):
                plt.scatter(result[i][0],result[i][1],s=30)
                newPos = listChangedPolygons[i]
                for i in range(len(shape)):
                    shape[i].set_xy(plt.Polygon(newPos[i]).get_xy())
                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.1)
        plt.pause(50)
    #Tìm các điểm lân cận của pos
    #Input: tọa độ pos
    #Output: danh sách các điểm lân cận, ngoại trừ các điểm trên polygons hoặc đi xuyên qua polygons
    def get_neighbours(self, pos):
        n = []
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
            x2 = pos[0] + dx
            y2 = pos[1] + dy
            if x2 < 0 or x2 > self.size[0]-1 or y2 < 0 or y2 > self.size[1]-1 or self.NuocDiHopLe(pos,(x2,y2))==0:
                continue
            n.append((x2, y2))
        return n

    #Tìm chi phí đường đi (g) từ điểm A đến điểm lân cận B
    #g = 1 nếu AB là ngang hoặc dọc
    #g = 1,41 nếu AB là đương chéo
    def move_cost(self, A, B):
        #if B in self.polygonPoint():
        #    return 99999 
        if A[0]==B[0] or A[1]==B[1]: 
            return 1
        return 1.41

    #Định nghĩa hàm heuristic (h')
    #Tính theo khoảng cách euclid từ pos tới điểm đích
    def heuristic(self,pos):
        return math.sqrt((pos[0]-self.goal[0])**2 + (pos[1]-self.goal[1])**2)
    def NuocDiHopLe(self, Pointcur, Pointnext):
        c=[]
        root=[]
        dt=Vietptdt(Pointcur,Pointnext)
        result=allduongthang(self.polygons)
        length=len(result[0][0])
    
        for x in range(0,length):
            k=giaihept(dt[0],result[0][0][x][0])
            root.append(k)
        # return root
        for x in range(0,length):
            check_pc=hople(root[x][0][0],Pointcur[0],Pointnext[0])
            check_pn=hople(root[x][0][1],Pointcur[1],Pointnext[1])
            i=hople(root[x][0][0],result[0][1][x][0],result[0][1][x][1]) #xét xem x có hợp lệ ko 
            j=hople(root[x][0][1],result[0][2][x][0],result[0][2][x][1])
            if(i==1 and j==1 and check_pc==1 and check_pn==1):
                return 0
        return 1



#Định nghĩa 1 node trên cây tìm kiếm
class _Node:
    #Constructor
    def __init__(self,coord,father,cost=None,heuristic=None,f=None):
        self.coord=coord            #Tọa độ trên bản đồ
        self.father=father          #Node cha
        self.cost=cost              #Chi phí đi từ điểm bắt đầu tơi node đang xét
        self.heuristic=heuristic    #h' tại node đang xét
        self.f=f #f=g+h: Dùng cho thuật toán A*

#Thuật giải A*
#Trả về: danh sách các điểm trên đường đi tìm được, chi phí đường đi
def AstarSearch(graph):
    start=_Node(graph.start, None, 0, graph.heuristic(graph.start), graph.heuristic(graph.start))
    # khởi tạo hàng đợi ưu tiên, và tập đóng
    prioQ=[start] 
    close=[]
    while True:
        #Nếu hàng đợi rỗng: A* ko tìm thấy đường đi
        if len(prioQ) == 0:
            return [],0
        #Lấy Node có f thấp nhất khỏi hàng đợi, đưa vào tập đóng
        CurNode=prioQ.pop(minNodeAstar(prioQ))
        close.append(CurNode)
        #Nếu Node đang xét có tọa độ trùng với điểm đích: return kết quả
        if CurNode.coord == graph.goal:
            return getPath(CurNode),CurNode.cost
        #Tạo tra tất cả các trạng thái kế tiếp của Node đang xét
        neighbours=graph.get_neighbours(CurNode.coord)
        for nei in neighbours:
            sub_cost = CurNode.cost + graph.move_cost(CurNode.coord, nei) 
            sub_heuristic = graph.heuristic(nei)
            f = sub_cost + sub_heuristic #f = g + h
            sub=_Node(nei, CurNode, sub_cost, sub_heuristic, f)
            #Nếu tồn tại 1 node trong close trùng trạng thái với sub
            isClose = False
            for node in close:
                if node.coord == sub.coord:
                    isClose=True
                    break
            if isClose == True: continue
            #So sánh trạng thái các node trong hàng đợi ưu tiên với node con
            isOpen = False
            for node in prioQ:
                #Nếu tồn tại 1 node trùng trạng thái với node con
                if node.coord==sub.coord:
                    isOpen = True
                    #Nếu f của node lớn hơn node con: cập nhật node cha, cost, f của node
                    if node.f > sub.f:
                        node.father=sub.father
                        node.cost=sub.cost
                        node.f=sub.f
                    break
            if isOpen == True: continue
            #Bổ sung node con vào hàng đợi ưu tiên:
            prioQ.append(sub)

#Thuật giải A* cho 2 điểm 
#Trả về: danh sách các điểm trên đường đi tìm được, chi phí đường đi
def AstarSearchFor2Points(graph, start, goal):
    # khởi tạo hàng đợi ưu tiên, và tập đóng
    start=_Node(start, None, 0, 0, 0)
    prioQ=[start] 
    close=[]
    while True:
        #Nếu hàng đợi rỗng: A* ko tìm thấy đường đi. Chi phí đường đi rất lớn
        if len(prioQ) == 0:
            return [],9999
        #Lấy Node có f thấp nhất khỏi hàng đợi, đưa vào tập đóng
        CurNode=prioQ.pop(minNodeAstar(prioQ))
        close.append(CurNode)
        #Nếu Node đang xét có tọa độ trùng với điểm đích: return kết quả
        if CurNode.coord == goal:
            return getPath(CurNode),CurNode.cost
        #Tạo tra tất cả các trạng thái kế tiếp của Node đang xét
        neighbours=graph.get_neighbours(CurNode.coord)
        for nei in neighbours:
            sub_cost = CurNode.cost + graph.move_cost(CurNode.coord, nei) 
            sub_heuristic = graph.heuristic(nei)
            f = sub_cost + sub_heuristic #f = g + h
            sub=_Node(nei, CurNode, sub_cost, sub_heuristic, f)
            #Nếu tồn tại 1 node trong close trùng trạng thái với sub
            isClose = False
            for node in close:
                if node.coord == sub.coord:
                    isClose=True
                    break
            if isClose == True: continue
            #So sánh trạng thái các node trong hàng đợi ưu tiên với node con
            isOpen = False
            for node in prioQ:
                #Nếu tồn tại 1 node trùng trạng thái với node con
                if node.coord==sub.coord:
                    isOpen = True
                    #Nếu f của node lớn hơn node con: cập nhật node cha, cost, f của node
                    if node.f > sub.f:
                        node.father=sub.father
                        node.cost=sub.cost
                        node.f=sub.f
                    break
            if isOpen == True: continue
            #Bổ sung node con vào hàng đợi ưu tiên:
            prioQ.append(sub)
#Con trỏ quay lui
#Input: Node đích đã được lấy ra khỏi container (hàng đợi ưu tiên, hàng đợi thường, hoặc stack)
#Trả vê: danh sách các điểm trên đường đi tìm được
def getPath(Node):
    result = [Node.coord]
    while True:
        father=Node.father
        if father == None:
            result.reverse()
            return result
        result.append(father.coord)
        Node = father
#Tìm kiếm node cho f nhỏ nhất
def minNodeAstar(lst):
    if lst == []: return None
    min = 0
    for i in range(len(lst)):
        if lst[i].f < min:
            min = i
    return min

#Thuật giải Breadth first search
#Trả về: danh sách các điểm trên đường đi tìm được, chi phí đường đi
def BreadthFS(graph):   
    start=_Node(graph.start,None,0)
    #Khởi tạo hàng đợi, tập close
    queue=[start]
    close=[]
    while True:
        #Nếu hàng đợi rỗng: BFS không tìm ra lời giải
        if queue == []:
            return [],0
        #Lấy node đầu tiên ra khỏi hàng đợi
        CurNode = queue.pop(0)

        #Nếu CurNode có trạng thái trùng với điểm đích: Return kq
        if CurNode.coord == graph.goal:
            return getPath(CurNode),CurNode.cost
        for closeNode in close:
            #nếu trạng thái CurNode đã có trong close: bỏ qua 
            if CurNode.coord == closeNode.coord:
                break
        else:
            #nếu ko: mở CurNode, đưa tất cả node con vào queue, bổ sung CurNode vào tập close
            #print(CurNode.coord)
            for nei in graph.get_neighbours(CurNode.coord):
                queue.append(_Node(nei, CurNode, CurNode.cost + graph.move_cost(CurNode.coord, nei)))
            close.append(CurNode)
            
#Thuật giải Depth first search
#Trả về: danh sách các điểm trên đường đi tìm được, chi phí đường đi
def DepthFS(graph):
    #return result,cost
    start=_Node(graph.start,None,0)
    #Khởi tạo stack, tập close
    stack=[start]
    close=[]
    while True:
        #Nếu stack rỗng: DFS không tìm ra lời giải
        if stack == []:
            return [],0
        #Lấy node trên cùng ra khỏi stack
        CurNode = stack.pop()

        #Nếu CurNode có trạng thái trùng với điểm đích: Return kq
        if CurNode.coord == graph.goal:
            return getPath(CurNode),CurNode.cost
        for closeNode in close:
            #nếu trạng thái CurNode đã có trong close: bỏ qua 
            if CurNode.coord == closeNode.coord:
                break
        else:
            #nếu ko: mở CurNode, đưa tất cả node con vào stack, bổ sung CurNode vào tập close
            #print(CurNode.coord)
            for nei in graph.get_neighbours(CurNode.coord):
                stack.append(_Node(nei, CurNode, CurNode.cost + graph.move_cost(CurNode.coord, nei)))
            close.append(CurNode)


def hople(x,x1,x2):   # nếu x thuộc trong khoảng x1,x2 thì thỏa ,x khoogn thuộc trong khoảng x[1][x2] thì hkoong thỏa
    if((x>=x1 and x<=x2)or(x>=x2 and x<=x1)):
        return 1
    else:
        return 0
def Vietptdt(x,y):  # hàm này là dùng để viết phương trình đường thăng với 2 điểm cho trước 
    x_n=y[0]-x[0]          #(3,4), (7,1) -> (4,-3)  -3(X-3)-4(Y-4)=-3X-4Y=-25
    y_n=y[1]-x[1] # tinh vec to chi phuong 
    a=y_n
    b=-x_n  #doi thanh vec tor phap tuyen
    z=a*(-x[0])+b*(-x[1]) # tinh he so tu do trong phuong trinh ax +b=c
    c=[]
    c.append((a,b,-z))
    return c
def Pointofline(Point ,line):  # hàm này để xét xem điểm có nằm trên đường thẳng hay không
    if((line[0]*Point[0]+line[1]*Point[1])==line[2]):
        return 1 # co nam tren duong thang
    else:
        return 0 # ko nằm trên đường thẳng 
def giaihept(a,b):# voi a, b, là 2 phương trình đường thẳng
    c=[]
    Det = a[0]*b[1]-b[0]*a[1]
    DetX=a[2]*b[1]-b[2]*a[1]
    DetY=a[0]*b[2]-b[0]*a[2]
    if(Det==0 and DetX==0 and DetY==0):# he phuong trinh vo so nghiem 
        c.append((-999,0))
        return c
    elif(Det==0 and DetX!=0 and DetY!=0):#he phuong trinh vo nghiem
        c.append((999,0))
        return c
    elif(Det==0 and DetX==0 and DetY!=0):
        c.append((999,0))
        return c
    elif(Det==0 and DetX!=0 and DetY==0):
        c.append((999,0))
        return c
    elif(Det!=0):
        x=DetX/Det
        y=DetY/Det
        c.append((x,y))
        return c #tra ve nghiem cua he phuong trinh 
def allduongthang(a): # hàm này mục tiêu trả về cạnh đang xét  trong tập cạnh với a là polygon
    c=[]
    e=[]
    dx=[]
    dy=[]
    result=[]
    khoangcach=[]
    for x in range(0,len(a)):
        for i in range(0, len(a[x])-1):
            d=Vietptdt(a[x][i],a[x][i+1])
            c.append(d)
            dx.append((a[x][i][0],a[x][i+1][0]))
            dy.append((a[x][i][1],a[x][i+1][1]))
        k=Vietptdt(a[x][0],a[x][len(a[x])-1])
        c.append(k)
        dx.append((a[x][0][0],a[x][len(a[x])-1][0]))
        dy.append((a[x][0][1],a[x][len(a[x])-1][1]))
    result.append((c,dx,dy))
    return result

#Hàm đọc file
#Input: đường dẫn file input
#Output: size, start, goal, polygons, pickup
def readFile(fileName):
    fobj = open(fileName)
    content=list(fobj)
    fobj.close()
    contentList=[]
    #Split
    for str in content:
        lineList = str.split(',')
        lenLineList = len(lineList)
        lineList[lenLineList - 1] = lineList[lenLineList - 1].rstrip('\n')
        contentList.append(lineList)
    #Get size, start, goal
    size = (int(contentList[0][0]),int(contentList[0][1]))
    start = (int(contentList[1][0]),int(contentList[1][1]))
    goal = (int(contentList[1][2]),int(contentList[1][3]))

    #Get tập các điểm đoán: pickup
    pickup = []
    lenSG = len(contentList[1])
    if lenSG > 4:
        pickup = [(int(contentList[1][i]),int(contentList[1][i+1])) for i in range(4, lenSG-1, 2)]
    
    #Get polygons
    polygons = []
    numPolygons = int(contentList[2][0])
    for i in range(3, numPolygons + 3):
        polygons.append([(int(contentList[i][j]),int(contentList[i][j+1])) for j in range(0, len(contentList[i]) - 1, 2)])

    return size,start,goal,polygons,pickup 

def changePosPolygons(originalPos, Numstate):
    dyEven=[-1,0,1,0]
    dyOdd=[1,0,-1,0]
    
    index = Numstate % len(dyEven)
    newPos = []
    for i in range(len(originalPos)):
        if i%2 == 0:
            newPos.append([(x, y + dyEven[index]) for x,y in originalPos[i]])
        else:
            newPos.append([(x, y + dyOdd[index]) for x,y in originalPos[i]])
    return newPos

#start,goal : kiểu điểm (x,y : kiểu int)
def EuclideanDistance(start,goal):
    return math.sqrt((start[0]-goal[0])**2 + (start[1]-goal[1])**2)
# truyền vào graph và danh sách các điểm đón
# trả về list thứ tự đường đi từ start đến goal
# sử dụng thuật toán gì đó để heuristic đường đi
# pickup kiểu list chứa các điểm
def FindTheWay(graph,pickup):

    result = []# kết quả trả về
    result.append(graph.start)# thêm start vào kết quả
    # start = _Node(graph.start,None,0,0,0)
    current_pickup =[graph.start]# gán vị trí hiện tại bằng start

    #duyệt đến khi hết list pickup
    while len(pickup)>0:
        minCost = 99999
        curPos = -1
        # duyệt tìm tất cả khoảng cách từ current_node với tất cả node còn lại trong pickup
        # dự đoán điểm gần nhất. Thêm điểm đó vào tập kết quả. Xóa điểm đó khỏi tập đang xét
        for i in range(0,len(pickup)):
            cost = EuclideanDistance(current_pickup[0],pickup[i])
            if cost<minCost:
                minCost = cost
                curPos = i
        # thêm vị trí vừa tìm được vào dãy
        result.append(pickup[curPos])
        # đổi vị trí hiện tại bằng vị trí mới
        current_pickup.pop()
        current_pickup.append(pickup[curPos])
        # xóa phần tử vừa xét khỏi tập xét
        pickup.pop(curPos)

    #thêm điểm goal vào
    result.append(graph.goal)
    return result



# pickup kiểu list các điểm 
def BruteForce(graph, pickup):
    result = []# kết quả
    cost = 0# chi phí đường đi

    order = FindTheWay(graph,pickup)# thứ tự đi từ start đến goal

    # duyệt qua list thứ tự. lần lượt tính khoảng cách theo đúng thứ tự. 
    while len(order)-2>= 0:
        start = order[0]
        goal = order[1]
        result_tmp, cost_tmp = AstarSearchFor2Points(graph,start,goal)
        result = result + result_tmp
        cost = cost + cost_tmp
        order.pop(0)

    return result,cost



#Hàm thay đổi vị trí của các đa giác (theo random có tính toán)
#Input: Graph, vị trí hiện tại của đa giác, vị trí hiện tại của điểm trên đường đi
#Output: vị trí mới của polygons
def ChangePolygonsPositions(graph, curPolygons, curPoint):
    nextPolygons = []
    for polygon in curPolygons:
        min = - 0.4
        max = 0.4
        count = 0
        while True:
            count +=1
            if count == 17: 
                nextPolygons.append(polygon.copy())
                break
            #Random độ dời của Polygons dx, dy trong đoạn [min, max]
            limit = np.arange(min, max, 0.1)
            dx = round(choice(limit), 2)
            dy = round(choice(limit), 2)

            #nếu không thay đổi vị trí
            if dx == 0 and dy == 0:
                nextPolygons.append(polygon.copy())
                break
            newPolygon = [(x + dx, y + dy) for x,y in polygon]
            #Nếu đa giác mới không "đè lên" curPoint, Goal, không đi vượt biên thì chấp nhận
            #Xét sự thay đổi vị trí tương đối giữa điểm hiện tại và đa giác
            #Xem đa giác là đứng yên, "sự chuyển dời của curPolygons so với curPoint" tương đương với "curPoint chuyển dời 1 đoạn -dx, -dy so với curPolygons"
            newPoint = (curPoint[0] - dx, curPoint[1] - dy)
            if CheckIntersection([polygon], curPoint, newPoint)  == 0:
                continue
            if CheckIntersection([polygon], graph.goal, (graph.goal[0] - dx, graph.goal[1] - dy)) == 0:
                continue
            isOut = 0
            for vertice in newPolygon:
                if vertice[0] < 0 or vertice[0] > graph.size[0] or vertice[1] < 0 or vertice[1] > graph.size[1]:
                    isOut = 1
                    break
            if isOut == 1: continue

            nextPolygons.append(newPolygon)
            break

    return nextPolygons

#Thuật giải A* tìm đường đi với map thay đổi theo thời gian
def AstartForAnimation(graph):
    copyGraph = _Graph(graph.size,graph.start,graph.goal,graph.polygons.copy(),graph.pickup.copy())
    listChangedPolygons  = [copyGraph.polygons]

    result,cost = AstarSearch(graph)
    if result == []:
        return [],0,listChangedPolygons
    if len(result) == 1:
        return [(graph.start)],0,listChangedPolygons

    finalResult = [copyGraph.start, result[1]]
    finalCost = copyGraph.move_cost(finalResult[len(finalResult) - 1], result[0])
    curPolygons = copyGraph.polygons.copy()
    while True:   
        nextPolygons = ChangePolygonsPositions(copyGraph, curPolygons, finalResult[len(finalResult) - 1])
        listChangedPolygons.append(nextPolygons)
        copyGraph.setPolygons(nextPolygons)
        curPolygons = nextPolygons.copy()

        result,cost = AstarSearchFor2Points(copyGraph, finalResult[len(finalResult) - 1], copyGraph.goal)
        if result == []:
            finalResult.append(finalResult[len(finalResult) - 1])
        elif len(result) >=2 :
            finalResult.append(result[1])
            finalCost += copyGraph.move_cost(finalResult[len(finalResult) - 1], result[0])
        else:
            return finalResult,finalCost,listChangedPolygons

            



#Kiểm tra sự giao nhau đoạn thằng từ PointCur tới PointNext với bất kì 1 cạnh trong polygons
#Output: True nếu không cắt nhau, False nếu cắt ít nhất 1 cạnh
def CheckIntersection(polygons, Pointcur, Pointnext):
    c=[]
    root=[]
    dt=Vietptdt(Pointcur,Pointnext)
    result=allduongthang(polygons)
    length=len(result[0][0])
    
    for x in range(0,length):
        k=giaihept(dt[0],result[0][0][x][0])
        root.append(k)
    # return root
    for x in range(0,length):
        check_pc=hople(root[x][0][0],Pointcur[0],Pointnext[0])
        check_pn=hople(root[x][0][1],Pointcur[1],Pointnext[1])
        i=hople(root[x][0][0],result[0][1][x][0],result[0][1][x][1]) #xét xem x có hợp lệ ko 
        j=hople(root[x][0][1],result[0][2][x][0],result[0][2][x][1])
        if(i==1 and j==1 and check_pc==1 and check_pn==1):
            return 0
    return 1   




script, __filename,__algorithm = argv
def main():
    #Khởi tạo bản đồ
    fileName = __filename
    #size=(10,8)
    #start=(2,2)
    #goal=(9,5)
    #polygons=[[(5,7),(4,3),(2,5)],[(5,1),(5,2),(8,2),(8,1)],[(6,4),(9,3),(7,6),(6,5)]]
    #pickup = [(3,3),(9,1),(1,7),(9,7),(2,6)]

    #fileName = "Input0.txt"
    #fileName = "Input1.txt"
    #fileName = "Input2.txt"
    #fileName = "Input3.txt"
    #fileName = "Input4.txt"
    #fileName = "Input5.txt"
    #fileName = "Input6.txt"
    size,start,goal,polygons,pickup = readFile(fileName)
    graph=_Graph(size, start, goal, polygons, pickup)

    listChangedPolygons = []
    #Gọi hàm tìm kiếm đường đi tại đây

    if pickup == []:
        if __algorithm == "0":
            result,cost = AstarSearch(graph)
        elif __algorithm == "1":
            result,cost = BreadthFS(graph)
        elif __algorithm == "2":
            result,cost = DepthFS(graph)
        elif __algorithm == "4":
            result,cost,listChangedPolygons = AstartForAnimation(graph)
    elif __algorithm == "3":
        result,cost = BruteForce(graph,pickup.copy())
    else:
       print("Invalid argv")
       return
    
    print("Chi phí:",cost)
    print("Đường đi:",result)
    #result=[]

    #Xuất kết quả
    #graph.draw(result)
    graph.drawAnimate(result,listChangedPolygons)

main()

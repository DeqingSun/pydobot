import math

class PCBMap:
    def __init__(self):
        self.pcbRefPoints = []
        self.dobotRefPoints = []
        self.dobotHeight = 20.3
        
        self.testPoints = []
        self.testPoints2 = []
        
    
    def addPcbRefPoint(self,x,y):
        self.pcbRefPoints.append((x,y));
        
    def addDobotRefPoint(self,x,y):
        self.dobotRefPoints.append((x,y));
        
    def calculateTransform(self):
        #for better algorithm
        #nghiaho.com/?page_id=671
        #https://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to
        if (len(self.pcbRefPoints)==len(self.dobotRefPoints) and len(self.pcbRefPoints)==2):
            #pcbCenterX = pcbCenterY = 0;
            #for p in self.pcbRefPoints: 
            #    pcbCenterX += p[0]
            #    pcbCenterY += p[1]
            #pcbCenterX = pcbCenterX/len(self.pcbRefPoints)
            #pcbCenterY = pcbCenterY/len(self.pcbRefPoints)
            
            
            #self.offset = (pcbCenterX,pcbCenterY)
            
            self.offset = (self.dobotRefPoints[0][0]-self.pcbRefPoints[0][0],self.dobotRefPoints[0][1]-self.pcbRefPoints[0][1]);
            anglePCB = math.atan2(self.pcbRefPoints[1][1]-self.pcbRefPoints[0][1],self.pcbRefPoints[1][0]-self.pcbRefPoints[0][0])
            angleDobot = math.atan2(self.dobotRefPoints[1][1]-self.dobotRefPoints[0][1],self.dobotRefPoints[1][0]-self.dobotRefPoints[0][0])
            self.angleDiff = angleDobot-anglePCB
            #print("OK",self.offset,anglePCB,self.angleDiff)
    
    def calculateMappedPoint(self,point):
        x=point[0]
        y=point[1]
        angle = self.angleDiff
        x1=math.cos(angle)*x-math.sin(angle)*y;
        y1=math.cos(angle)*y+math.sin(angle)*x;
        return (x1+self.offset[0],y1+self.offset[1])
    
    def calculateMappedPoint(self,point):
        x=point[0]
        y=point[1]
        angle = self.angleDiff
        x1=math.cos(angle)*x-math.sin(angle)*y;
        y1=math.cos(angle)*y+math.sin(angle)*x;
        return (x1+self.offset[0],y1+self.offset[1])
    
    def testFunc(self):
        for i in range(24):
            x=i%10;
            y=i
            x=x*2.54
            y=y*2.54
            self.testPoints.append((x,y));
        
        #print(self.testPoints)
        
        for p in self.testPoints: 
            self.testPoints2.append(self.calculateMappedPoint(p));

if __name__ == '__main__':
    pcbMap = PCBMap()
    
    pcbMap.addPcbRefPoint(0,0);
    pcbMap.addPcbRefPoint(2.54*9,2.54*23);
    
    pcbMap.addDobotRefPoint(240.5,-0.7);
    pcbMap.addDobotRefPoint(225.0,60.4);
    
    pcbMap.calculateTransform()
    
    pcbMap.testFunc()
    
    print(pcbMap.pcbRefPoints)
    print(pcbMap.dobotRefPoints)
    
    print(pcbMap.calculateMappedPoint(pcbMap.pcbRefPoints[0]) )
    print(pcbMap.calculateMappedPoint(pcbMap.pcbRefPoints[1]) )

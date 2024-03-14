import numpy as np

# refposedeneme=np.array([[0,0],[1,1],[2,2],[3,3],[4,4],[5,5],[6,6],[7,7]])
# currposedeneme=np.array([[2,2],[4,4],[6,6]])

class TrajectoryProcess:
    def __init__(self,refpose):
        self.refpose=refpose
        self.segmentsize=50
        self.i=0
        self.min_distance_index=0
        self.k_st=10
        self.incstep = 30
    def calculate_min_distance(self,currentpose):
        distances=np.linalg.norm(self.refpose[self.i:self.i+self.segmentsize,:]-currentpose,axis=1)
        self.min_distance_index=np.argmin(distances)
        #print (np.amin(distances),np.argmin(distances))
        return np.amin(distances)
    
        
    def calculate_heading(self):
        ref_heading=np.arctan(self.refpose[self.i+1,1]-self.refpose[self.i,1])/(self.refpose[self.i+1,0]-self.refpose[self.i,0])
        #current heading is measured from car's sensors
        
        return ref_heading
    
    def process_poses_stanley(self,currentpose,current_heading,v):
        cross_track_error=self.calculate_min_distance(currentpose)
        ref_heading=self.calculate_heading()
        #heading error calcs

        heading_error=ref_heading - current_heading
        if heading_error > np.pi:
                heading_error -= 2 * np.pi
        if heading_error < - np.pi:
            heading_error+= 2 * np.pi
        
        print(heading_error)
        ########
        ####      crosstrack calcs
        yaw_cross_track = np.arctan2(currentpose[1]-self.refpose[self.i+self.min_distance_index][1], currentpose[0]-self.refpose[self.i+self.min_distance_index][0])
        yaw_path2ct = ref_heading - yaw_cross_track
        if yaw_path2ct > np.pi:
            yaw_path2ct -= 2 * np.pi
        if yaw_path2ct < - np.pi:
            yaw_path2ct += 2 * np.pi
        if yaw_path2ct > 0:
            cross_track_error = abs(cross_track_error)
        else:
            cross_track_error = - abs(cross_track_error)
        # yaw_diff_crosstrack = np.arctan(k_e * cross_track_error / (v))



        steercmd=heading_error + np.arctan((self.k_st*cross_track_error)/(v+0.00001))
        if steercmd > np.pi:
            steercmd -= 2 * np.pi
        if steercmd < - np.pi:
            steercmd += 2 * np.pi
        steercmd = min(1.22, steercmd)
        steercmd = max(-1.22, steercmd)
        # delta=max(-np.pi/6, min(steercmd,np.pi/6))
        # print(delta)
        if self.min_distance_index == self.segmentsize-1:
            self.i+=self.incstep
        return steercmd
    
# if __name__ == "__main__":       
#     deneme=TrajectoryProcess(refposedeneme)
#     for a in currposedeneme:
#         print(deneme.process_poses_stanley(a,np.pi/4,5))
#         # print(deneme.process_poses(90))
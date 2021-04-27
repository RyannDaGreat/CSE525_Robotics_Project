#These heights and widths were measured with a ruler in real life
arena_height_in_cm=115
arena_width_in_cm=157
pixels_per_cm=10

def unwarp_arena_image(image,verbose=False):
    tags={}
    for tag in detect_apriltags(image):
        tags[tag.id_number]=tag
        
    if verbose:
        print("unwarp_arena_image: Detected apriltags ",sorted(tags))
    
    corners=[]
    for id_number in [0,1,2,3]:
        if id_number in tags:
            corners.append(tags[id_number].center)

    height=arena_height_in_cm*pixels_per_cm    
    width =arena_width_in_cm *pixels_per_cm    
   
    if len(corners)>=4:
        return unwarped_perspective_image(image,corners,height=height,width=width)
    else:
        raise Exception('unwarp_arena_image: Failed to find all four apriltags')
    
def test_unwarper():
    while True:
        image=load_image_from_webcam(1)
        try:
            image=unwarp_arena_image(image)
        except Exception:
            pass
        display_image(image)
        
camera_index=4 if get_computer_name()=='mohiiiiiib-OP-LP3' else 2 #On Ryan's MacBook, the camera index is 2. On Mohib's laptop, the camera index is 4.
def load_image_from_arena():
    for _ in range(3):
        load_image_from_webcam(4)
    sleep(.1)#Get rid of annoying camera lag
    return load_image_from_webcam(4) #Configure this to the appropriate webcam
def do_action(action):
    shell_command('sshpass -p a ssh -t eve@walle-desktop.local \'echo "%s()" > /home/eve/CleanCode/Robot/commands/command.py\''%action)
class ArenaState:
    def __init__(self):
        self.arena_image=load_image_from_arena()
        self.unwarped=unwarp_arena_image(self.arena_image)
        self.tags=detect_apriltags(self.unwarped)
        self.robot_tags=[tag for tag in self.tags if tag.id_number==5]
        assert len(self.robot_tags)==1,'Cannot find robot'
        self.robot_tag=next(iter(self.robot_tags))
        self.robot_center=self.robot_tag.center
        self.forward_vec=normalized(self.robot_tag.corners[3]-self.robot_tag.corners[2])
        self.robot_angle=(np.arctan2(*self.forward_vec)/tau*360-90)%360
        self.cyan_block_tags=[tag for tag in self.tags if tag.id_number==98]
        assert len(self.cyan_block_tags)==1,'Cannot cyan block'
        self.cyan_block_tag=next(iter(self.cyan_block_tags))
        self.cyan_block_center=self.cyan_block_tag.center
        self.robot_to_block_vector=self.cyan_block_center-self.robot_center
        self.robot_to_block_angle=((np.arctan2(*self.robot_to_block_vector)/tau*360-90)-self.robot_angle)%360
        if self.robot_to_block_angle>180:
            self.robot_to_block_angle=-(360-self.robot_to_block_angle)
        self.distance_to_cyan_block_in_cm=euclidean_distance(self.robot_center,self.cyan_block_center)/pixels_per_cm
        
    def display(self):
        icecream.ic(self.robot_angle,self.robot_to_block_angle,self.distance_to_cyan_block_in_cm)
        image=self.unwarped
        image=cv_draw_contour(image,contour=self.robot_tag.corners,copy=True,color=(0,0,0),width=6)
        image=cv_draw_contour(image,contour=self.robot_tag.corners,copy=True,color=(0,255,0),width=3)
        image=cv_draw_contour(image,contour=[self.robot_center,self.robot_center+80*self.forward_vec],copy=True,color=(0,0,0),width=6)
        image=cv_draw_contour(image,contour=[self.robot_center,self.robot_center+80*self.forward_vec],copy=True,color=(255,0,0),width=3)
        image=cv_draw_circle(image,*self.cyan_block_center.astype(int),radius=6,color=(0,0,0))
        image=cv_draw_circle(image,*self.cyan_block_center.astype(int),color=(0,255,255))
        display_image(image)
while True:
    try:
        a=ArenaState()
        a.display()
        
        if abs(a.robot_to_block_angle)>50:
            if a.robot_to_block_angle>0:
                do_action('left')
            elif a.robot_to_block_angle<0:
                do_action('right')
        elif a.distance_to_cyan_block_in_cm>20:
            do_action('forward')

    except Exception as e:
        display_image(load_image_from_arena())
        fansi_print("ERROR: "+str(e),'red','bold')
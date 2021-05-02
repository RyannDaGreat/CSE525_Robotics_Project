current_computer="MOHIB" if get_computer_name()=='mohiiiiiib-OP-LP3' else "MAC"
#Key:
#   "MAC" = Ryan's Macbook
#   "MOHIB" = Mohib's Ubuntu Laptop
#   "GLASS" = Ryan's Desktop

camera_index=4 if current_computer=='MOHIB' else 1 #On Ryan's MacBook, the camera index is 1. On Mohib's laptop, the camera index is 4.

#Arena Dimensions
arena_height_in_cm=115
arena_width_in_cm=157
pixels_per_cm=5

def load_image_from_arena():
    tic()
    while toc()<.5:
        load_image_from_webcam(camera_index)
    return load_image_from_webcam(camera_index) #Configure this to the appropriate webcam

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
    #Display an image stream of the arena
    while True:
        image=load_image_from_webcam(camera_index)
        try:
            image=unwarp_arena_image(image)
        except Exception:
            pass
        display_image(image)

class ArenaState:
    def __init__(self,block_center=None):
        self.arena_image=load_image_from_arena()
        self.unwarped=unwarp_arena_image(self.arena_image)
        self.tags=detect_apriltags(self.unwarped)

        self._robot_tags=[tag for tag in self.tags if tag.id_number==5]
        assert len(self._robot_tags)==1,'Cannot find robot'
        self.robot_tag=next(iter(self._robot_tags))
        self.robot_center=self.robot_tag.center
        self.robot_forward_vec=normalized(self.robot_tag.corners[3]-self.robot_tag.corners[2])
        self.robot_angle=(np.arctan2(*self.robot_forward_vec)/tau*360-90)%360

        if block_center is None:
            self._block_tags=[tag for tag in self.tags if tag.id_number==98]
            assert len(self._block_tags)==1,'Cannot cyan block'
            self._block_tag=next(iter(self._block_tags))
            self.block_center=self._block_tag.center
        else:
            # Sometimes we want to override this for the sake of moving the robot to a place we choose
            self.block_center=block_center

        self.robot_to_block_vector=self.block_center-self.robot_center
        self.robot_to_block_angle=((np.arctan2(*self.robot_to_block_vector)/tau*360-90)-self.robot_angle)%360
        if self.robot_to_block_angle>180:
            self.robot_to_block_angle=-(360-self.robot_to_block_angle)
        self.distance_to_block_in_cm=euclidean_distance(self.robot_center,self.block_center)/pixels_per_cm

    def display(self):
        icecream.ic(self.robot_angle,self.robot_to_block_angle,self.distance_to_block_in_cm)
        image=self.unwarped
        image=cv_draw_contour(image,contour=self.robot_tag.corners,copy=True,color=(0,0,0),width=6)
        image=cv_draw_contour(image,contour=self.robot_tag.corners,copy=True,color=(0,255,0),width=3)
        image=cv_draw_contour(image,contour=[self.robot_center,self.robot_center+80*self.robot_forward_vec],copy=True,color=(0,0,0),width=6)
        image=cv_draw_contour(image,contour=[self.robot_center,self.robot_center+80*self.robot_forward_vec],copy=True,color=(255,0,0),width=3)
        image=cv_draw_circle(image,*self.block_center.astype(int),radius=6,color=(0,0,0))
        image=cv_draw_circle(image,*self.block_center.astype(int),radius=5,color=(0,255,255))
        display_image(image)

class Actor:
    def next_action(state,):
        pass

class SimpleActor()
    def next_action_hardcoded(self):
        if abs(a.robot_to_block_angle)>30:
            if self.robot_to_block_angle>0:
                return 'left'
            elif self.robot_to_block_angle<0:
                return 'right'
        elif self.distance_to_block_in_cm>20:
            return 'forward'

class Robot:
    #This is a class for now because we may add more functions and more robots, such as a second robot and a load_image function to get images from the robot's camera
    def do_action(self,action:str):
        if not action:return #A null-action. Can be None or '' etc
        assert isinstance(action,str),'Action must be a string but got type '+repr(type(action))
        assert action in 'left right forward backward'.split(),'Invalid action: '+action
        shell_command('sshpass -p a ssh -t eve@walle-desktop.local \'echo "%s()" > /home/eve/CleanCode/Robot/commands/command.py\''%action)
    def do_random_action(self):
        self.do_action(random_element('forward backward'.split()))

robot=Robot()

while True:
    if random_chance(.1):
        robot.do_random_action()
    try:
        a=ArenaState()
        a.display()
        action=a.next_action_hardcoded()
        robot.do_action(action)
    except Exception as e:
        display_image(load_image_from_arena())
        print_verbose_stack_trace(e)
        fansi_print("ERROR: "+str(e),'red','bold')

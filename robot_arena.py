from rp import *
import json

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
    def __init__(self,*,unwarped_image=None,block_center_in_cm=None):
        #Properties:
        #   unwarped_image, robot_center_in_cm, robot_forward_vec, block_center_in_cm

        if unwarped_image is not None:
            self.unwarped_image=unwarped_image
        else:
            self._arena_image=load_image_from_arena()
            self.unwarped_image=unwarp_arena_image(self._arena_image)

        self._tags=detect_apriltags(self.unwarped_image)
        self._robot_tags=[tag for tag in self._tags if tag.id_number==5]
        assert len(self._robot_tags)==1,'Cannot find robot'
        self._robot_tag=next(iter(self._robot_tags))

        self.robot_center_in_cm=self._robot_tag.center/pixels_per_cm
        self.robot_forward_vec=normalized(self._robot_tag.corners[3]-self._robot_tag.corners[2])

        if block_center_in_cm is None:
            self._block_tags=[tag for tag in self._tags if tag.id_number==98]
            assert len(self._block_tags)==1,'Cannot find cyan block'
            self._block_tag=next(iter(self._block_tags))
            self.block_center_in_cm=self._block_tag.center/pixels_per_cm
        else:
            # Sometimes we want to override this for the sake of moving the robot to a place we choose
            self.block_center_in_cm=block_center_in_cm

    @property
    def robot_angle(self):
        return (np.arctan2(*self.robot_forward_vec)/tau*360-90)%360    

    @property
    def robot_to_block_vector_in_cm(self):
        return self.block_center_in_cm-self.robot_center_in_cm

    @property
    def robot_to_block_distance_in_cm(self):
        return magnitude(self.robot_to_block_vector_in_cm)
    
    @property
    def robot_to_block_angle(self):
        angle=((np.arctan2(*self.robot_to_block_vector_in_cm)/tau*360-90)-self.robot_angle)%360
        if angle>180:
            angle=-(360-angle)
        return angle

    @property
    def robot_center_in_pixels(self):
        return (self.robot_center_in_cm*pixels_per_cm).astype(int)
    
    @property
    def block_center_in_pixels(self):
        return (self.block_center_in_cm*pixels_per_cm).astype(int)
    
    def display(self):
        icecream.ic(self.robot_angle,self.robot_to_block_angle,self.robot_to_block_distance_in_cm)
        image=self.unwarped_image
        image=cv_draw_circle(image,*self.robot_center_in_pixels,radius=10,color=(0  ,0,0))
        image=cv_draw_circle(image,*self.robot_center_in_pixels,radius= 9,color=(255,0,0))
        image=cv_draw_contour(image,contour=[self.robot_center_in_pixels,self.robot_center_in_pixels+80*self.robot_forward_vec],copy=True,color=(0  ,0,0),width=6)
        image=cv_draw_contour(image,contour=[self.robot_center_in_pixels,self.robot_center_in_pixels+80*self.robot_forward_vec],copy=True,color=(255,0,0),width=3)
        image=cv_draw_circle(image,*self.block_center_in_pixels,radius=10,color=(0,  0,  0))
        image=cv_draw_circle(image,*self.block_center_in_pixels,radius=9 ,color=(0,255,255))
        display_image(image)

    def save(self,path,*,image_quality=100)->SavedArenaState:
        saved_arena_state=SavedArenaState(path,
                                          unwarped_image    =self.unwarped_image,
                                          robot_center_in_cm=self.robot_center_in_cm,
                                          robot_forward_vec =self.robot_forward_vec,
                                          block_center_in_cm=self.block_center_in_cm)
        saved_arena_state.save(image_quality)
        return saved_arena_state

class SavedArenaState(ArenaState):
    def __init__(self,*, 
                 path,
                 unwarped_image,
                 robot_center_in_cm,
                 robot_forward_vec,
                 block_center_in_cm):
        self.path              =path
        self.unwarped_image    =unwarped_image
        self.robot_center_in_cm=robot_center_in_cm
        self.robot_forward_vec =robot_forward_vec
        self.block_center_in_cm=block_center_in_cm
    
    @property
    def is_saved(self):
        return path_exists(self.path)

    @property
    def state_path(self):
        return path_join(self.path,'state.json')
    
    @property
    def image_path(self):
        return path_join(self.path,'image.jpg')
    
    def save(*,image_quality=100):
        # image_quality is measured in percent for the .jpg file
        if self.is_saved:
            return

        make_folder(self.path)

        state=dict()
        state['robot_center_in_cm']=robot_center_in_cm
        state['robot_forward_vec' ]=robot_forward_vec 
        state['block_center_in_cm']=block_center_in_cm

        import json
        json.dump(state,self.state_path)

        save_image_jpg(self.unwarped_image,self.image_path,quality=image_quality)

    @staticmethod
    def load(path:str)->SavedArenaState:
        assert is_a_folder(path)
        assert file_exists(self.state_path)
        assert file_exists(self.image_path)
        assert is_image_file(self.image_path)

        import json
        state=json.load(self.state_path)

        unwarped_image    =load_image(self.image_path)
        robot_center_in_cm=state['robot_center_in_cm']
        robot_forward_vec =state['robot_forward_vec' ]
        block_center_in_cm=state['block_center_in_cm']

        return SavedArenaState(unwarped_image    =unwarped_image,
                               robot_center_in_cm=robot_center_in_cm,
                               robot_forward_vec =robot_forward_vec,
                               block_center_in_cm=block_center_in_cm)

def get_current_arena_state(**kwargs):
    #Come hell or high water, this function WILL eventually get you the arena state...or else it will go into an infinite loop. It will not throw an error.
    while True:
        try:
            return ArenaState(**kwargs)
        except Exception as e:
            text_to_speech("crap!")
            display_image(load_image_from_arena())
            print_verbose_stack_trace(e)
            fansi_print("ERROR: "+str(e),'red','bold')

class Actor:
    def next_action(state):
        #An object that takes states and returns actions
        #Will later have a subclass that uses neural networks
        pass

class SimpleActor(Actor):
    def __init__(self,angle_threshold=30,distance_threshold=20):
        #angle_threshold is in degrees
        #distance_threshold is in centimeters
        self.angle_threshold=angle_threshold
        self.distance_threshold=distance_threshold

    def next_action(self,state:ArenaState):
        #If we're not in a terminal state, it will return an action
        #If it returns None, that means we're in a terminal state (aka the robot has reached its target). Yay!
        if abs(state.robot_to_block_angle)>self.angle_threshold:
            if state.robot_to_block_angle>0:
                return 'left'
            elif state.robot_to_block_angle<0:
                return 'right'
        elif state.robot_to_block_distance_in_cm>self.distance_threshold:
            return 'forward'
        else:
            return None #Let's do this explicitly

class Robot:
    #This is a class for now because we may add more functions and more robots, such as a second robot and a load_image function to get images from the robot's camera
    def do_action(self,action:str):
        if not action:return #A null-action. Can be None or '' etc
        assert isinstance(action,str),'Action must be a string but got type '+repr(type(action))
        assert action in 'left right forward backward'.split(),'Invalid action: '+action
        shell_command('sshpass -p a ssh -t eve@walle-desktop.local \'echo "%s()" > /home/eve/CleanCode/Robot/commands/command.py\''%action)
    def do_random_action(self):
        self.do_action(random_element('forward backward'.split()))

def chase_target(robot:Robot=robot,
                 actor:Actor=actor,
                 get_state=get_current_arena_state,
                 target_in_cm:np.ndarray=None):
    #Go to the target once then return

    if target_in_cm is None:
        #Choose a random target if none is specified
        target_in_cm=as_numpy_array([random_float(arena_height_in_cm),random_float(arena_width_in_cm)])

    fansi_print("Chasing target: "+repr(target_in_cm),'yellow','bold')

    assert len(target_in_cm),'Target should be a y,x coordinate pair'
    while True:
        state=get_state(block_center_in_cm=target_in_cm)
        state.display()
        action=actor.next_action(state)
        if action is None:
            fansi_print("Finished chasing target: "+repr(target_in_cm),'yellow','bold')
            return
        robot.do_action(action)

def chase_block():
    #Chase the physical blue block indefinitely
    while True:
        state=get_current_arena_state()
        state.display()
        action=actor.next_action(state)
        if action is not None:
            if random_chance(.1):
                robot.do_random_action()
            robot.do_action(action)
        else:
            fansi_print("Yay! We've reached the block!",'green','bold')
        text_to_speech("yay!")

robot=Robot()
actor=SimpleActor(distance_threshold=10)


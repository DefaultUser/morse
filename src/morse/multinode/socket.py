import logging; logger = logging.getLogger("morse." + __name__)
import socket
import pickle
import mathutils
from bpy import data as bpy_data

from morse.core.multinode import SimulationNodeClass

class SocketNode(SimulationNodeClass):
    """ 
    Implements multinode simulation using sockets.
    """

    node_socket = None
    connected = False
    out_data = {}
    pose_data = {}
    active_obj_data = {}

    def initialize(self):
        """
        Create the socket that will be used to commmunicate to the server.
        """
        logger.debug("Connecting to port %s:%d" % (self.host, self.port))
        try:
            self.node_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.node_socket.connect((self.host, self.port))
            logger.info("Connection established to node manager (%s, %s)" % (self.host, self.port))
            self.connected = True
        except socket.error as detail:
            logger.warning("Multi-node simulation not available!!")
            logger.info("\tUnable to connect to server: (%s, %s)" % (self.host, self.port))
            logger.info("\t%s" % detail)
            self.connected = False

    def _exchange_data(self, out_data, pose_data, obj_data):
        """ Send and receive pickled data through a socket """
        # Use the existing socket connection
        if self.connected:
            message = pickle.dumps([self.node_name, out_data, pose_data, obj_data])
            sock = self.node_socket
            sock.send(message)
            response = sock.recv(8192)
            in_data = pickle.loads(response)
            #logger.debug("Received: %s" % in_data)
            return (in_data)

    def synchronize(self):
        if self.connected:
            # Get the coordinates of local robots
            for obj, local_robot_data in self.gl.robotDict.items():
                #self.out_data[obj.name] = [obj.worldPosition.to_tuple()]
                euler_rotation = obj.worldOrientation.to_euler()
                self.out_data[obj.name] = [obj.worldPosition.to_tuple(), [euler_rotation.x, euler_rotation.y, euler_rotation.z]]
            # Get the pose of local armatures
            for component_name, local_robot_data in self.gl.componentDict.items():
                try:
                    if bpy_data.objects[component_name].type == 'ARMATURE':
                        scene = self.gl.getCurrentScene()
                        component = scene.objects[component_name]
                        if component.parent.get('Robot_Tag', False):
                            armature_data = {}
                            for bone in component.channels:
                                armature_data[bone.name] = [tuple(bone.location), tuple(bone.joint_rotation)]
                            self.pose_data[component.name] = armature_data
                except KeyError:
                    pass
            # Get the coordinates of selected objects
            self.active_obj_data = {}
            for obj, local_obj_data in self.gl.passiveObjectsDict.items():
                if obj.parent:
                    self.active_obj_data[obj.name] = [tuple(obj.worldPosition), tuple(obj.worldOrientation.to_euler()), self.node_name]
            # Send the encoded dictionary through a socket
            #  and receive a reply with any changes in the other nodes
            in_data = self._exchange_data(self.out_data, self.pose_data, self.active_obj_data)

            if in_data != None:
                # Update the positions of the external robots
                for obj_name, robot_data in in_data[0].items():
                    try:
                        obj = scene.objects[obj_name]
                        if obj not in self.gl.robotDict:
                            logger.debug("Data received: ", robot_data)
                            obj.worldPosition = robot_data[0]
                            obj.worldOrientation = mathutils.Euler(robot_data[1]).to_matrix()
                    except KeyError as detail:
                        logger.info("Robot %s not found in this simulation scenario, but present in another node. Ignoring it!" % detail)
                # Update the pose of external armamtures
                for armature_name, armature_data in in_data[1].items():
                    try:
                        armature = scene.objects[armature_name]
                        if armature.parent.get('External_Robot_Tag', False):
                            channels = armature.channels
                            for name, state in armature_data.items():
                                channels[name].location = mathutils.Vector(state[0])
                                channels[name].joint_rotation = state[1]
                            armature.update()
                    except KeyError as detail:
                        logger.info("Component %s not found in this simulation scenario, but present in another node. Ignoring it!" % detail)
                # Update externally selected objects
                for obj_name, obj_state in in_data[2].items():
                    try:
                        obj = scene.objects[obj_name]
                        if not obj.parent and self.node_name != obj_state[2]:
                            obj.worldPosition = obj_state[0]
                            obj.worldOrientation = mathutils.Euler(obj_state[1]).to_matrix()
                            obj.suspendDynamics()
                    except KeyError as detail:
                        logger.info("Passive Object %s not found in this simulation scenario, but present in another node. Ignoring it!" % detail)
                
                for obj, local_obj_data in self.gl.passiveObjectsDict.items():
                    if obj.name not in in_data[2]:
                        obj.restoreDynamics()

    def finalize(self):
        """ Close the communication socket. """
        self.node_socket.close()
        self.connected = False

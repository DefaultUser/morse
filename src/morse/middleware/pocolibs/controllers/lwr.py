from morse.middleware.pocolibs.controllers.Lwr_Poster import ors_lwr_poster

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    # Check if the name of the poster has been given in mw_data
    try:
        # It should be the 4th parameter
        poster_name = mw_data[3]
    except IndexError as detail:
        # Compose the name of the poster, based on the parent and module names
        poster_name = '{0}_{1}'.format(parent_name, component_name)

    poster_id, ok = ors_lwr_poster.locate_poster(poster_name)
    # Use the value of 'ok' to determine if the poster was found
    if ok != 0:
        print ("Located 'lwr' poster named '%s'. ID=%d" % (poster_name, poster_id))
        self._poster_in_dict[component_name] = poster_id
        component_instance.input_functions.append(function)
    else:
        print ("Poster 'lwr' not created. Component will not work")


def read_lwr_config(self, component_instance):
    """ Read the angles for the segments of the Kuka arm """
    # Read from the poster specified
    poster_id = self._poster_in_dict[component_instance.blender_obj.name]
    gbm_conf = ors_lwr_poster.read_lwr_data(poster_id)

    component_instance.modified_data[0] = gbm_conf.q1
    component_instance.modified_data[1] = gbm_conf.q2
    component_instance.modified_data[2] = gbm_conf.q3
    component_instance.modified_data[3] = gbm_conf.q4
    component_instance.modified_data[4] = gbm_conf.q5
    component_instance.modified_data[5] = gbm_conf.q6
    component_instance.modified_data[6] = gbm_conf.q7

    # Return true to indicate that a command has been received
    return True
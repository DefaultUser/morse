import GameLogic

class MorseNEDClass(object):
	""" Convert between ENU and NED coordinates. """
	
	def __init__(self, obj, parent=None):
		self.blender_obj = obj
		#self.init_components()


	def __del__(self):
		""" Destructor method. """
		pass


	def register_component(self, component_name, component_instance, mod_data):
		""" Add the corresponding function to a component. """
		# Extract the information for this modifier
		# This will be tailored for each middleware according to its needs
		function_name = mod_data[1]

		try:
			# Get the reference to the function
			function = getattr(self, function_name)
		except AttributeError as detail:
			print ("ERROR: %s. Check the 'component_config.py' file for typos" % detail)
			return

		# Choose what to do, depending on the function being used
		# Data read functions
		if function_name == "ned_to_blender":
			component_instance.input_modifiers.append(function)
		# Data write functions
		elif function_name == "blender_to_ned":
			component_instance.output_modifiers.append(function)


	def blender_to_ned(self, component_instance):
		""" Convert the coordinates from Blender to UTM reference. """
		component_instance.modified_data['x'] = component_instance.modified_data['y']
		component_instance.modified_data['y'] = component_instance.modified_data['x']
		component_instance.modified_data['z'] = -component_instance.modified_data['z']

		return component_instance.modified_data


	def ned_to_blender(self, component_instance):
		""" Convert the coordinates from UTM to Blender reference. """
		component_instance.modified_data['x'] = component_instance.modified_data['y']
		component_instance.modified_data['y'] = component_instance.modified_data['x']
		component_instance.modified_data['z'] = -component_instance.modified_data['z']

		return component_instance.modified_data
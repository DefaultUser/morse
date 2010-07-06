import GameLogic

class MorseUTMClass(object):
	""" Convert between Blender and UTM coordinates. """
	
	def __init__(self, obj, parent=None):
		""" Initialize the global UTM coordinates in the scene. """
		self.blender_obj = obj
		#self.init_components()

		self._global_x = 0.0
		self._global_y = 0.0
		self._global_z = 0.0

		# Get the global coordinates if defined in the scene
		scene = GameLogic.getCurrentScene()
		script_empty_name = 'Scene_Script_Holder'
		# Prefix the name of the component with 'OB'
		# Will only be necessary until the change to Blender 2.5
		if GameLogic.pythonVersion < 3:
			script_empty_name = 'OB' + script_empty_name
		script_empty = scene.objects[script_empty_name]
		self._global_x = float(script_empty['UTMXOffset'])
		self._global_y = float(script_empty['UTMYOffset'])
		self._global_z = float(script_empty['UTMZOffset'])


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
		if function_name == "utm_to_blender":
			component_instance.input_modifiers.append(function)
		# Data write functions
		elif function_name == "blender_to_utm":
			component_instance.output_modifiers.append(function)


	def blender_to_utm(self, component_instance):
		""" Convert the coordinates from Blender to UTM reference. """
		#component_instance.modified_data['x'] += self._global_x
		#component_instance.modified_data['y'] += self._global_y
		#component_instance.modified_data['z'] += self._global_z
		component_instance.modified_data[0] += self._global_x
		component_instance.modified_data[1] += self._global_y
		component_instance.modified_data[2] += self._global_z

		return component_instance.modified_data


	def utm_to_blender(self, component_instance):
		""" Convert the coordinates from UTM to Blender reference. """
		#component_instance.modified_data['x'] -= self._global_x
		#component_instance.modified_data['y'] -= self._global_y
		#component_instance.modified_data['z'] -= self._global_z
		component_instance.modified_data[0] -= self._global_x
		component_instance.modified_data[1] -= self._global_y
		component_instance.modified_data[2] -= self._global_z

		return component_instance.modified_data
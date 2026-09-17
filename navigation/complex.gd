##node containing all spatial nodes and managing the navigation system
@tool class_name Complex extends Node

#TODO: set rendering resources
const POINT_MESH: SphereMesh = preload("uid://...") ##the mesh rendered for the points on the navigation grid
const CONNECTION_MATERIAL: StandardMaterial3D = preload("uid://...") ##the material rendered onto a path line on the navigation grid
const COLOR_DISABLED: Color = Color(1.0, 1.0, 1.0, 1.000) ##color of points on the navigation grid that are disabled
const COLOR_DEFAULT:  Color = Color(1.0, 1.0, 1.0, 0.122) ##color of points on the navigation grid by default
const COLOR_PATH:     Color = Color(1.0, 0.0, 0.0, 1.000) ##color of points on the navigation grid that are also on a path line
const ADVANCE_THRESHOLD: float = 0.5 ##the minimum distance an agent has to be to a point on their path line to get the next point on the line
const DEVIATION_THRESHOLD: float = 0.5 ##if an agent is this many multiples of spacing from it's next point, the path recomputes
const REFRESH_DELAY: float = 0.1 ##the time, in seconds, between refreshes

@export var size: int = 1: ##the width/height/depth of the grid, # of points is size ** 3
	set(v): #regenerates grid of every set
		if size == v: return
		size = v
		generate_grid()
@export_custom(PROPERTY_HINT_NONE, "suffix:m") var spacing: float = 1.0: ##the distance inbetween points
	set(v): #regenerates grid of every set
		if spacing == v: return
		spacing = v
		generate_grid()
@export_tool_button("Reset") var reset: Callable = _reset ##calls the reset function, clearing all associated nodes and caches

var _astar: AStar3D = AStar3D.new() ##the navigation brain storing the grid of points
#NOTE: yes I understand very well that the boolean value is never used whatsoever but Dictionarys
#      run faster than Arrays, and since there's a lot of points, performance is everything!!
var _disabled: Dictionary[int, bool] ##cache of the points that were disabled between refreshes

var _shapes: Dictionary[CollisionShape3D, Shape] ##a cache of the shape states of registered CollisionShape3Ds in the scene
var _shape_add: Callable = func(node: Node) -> void: ##adds a CollisionShape3D to the shapes cache paired to a generated Shape state
	if node is CollisionShape3D: 
		var shape: CollisionShape3D = node
		_shapes[shape] = Shape.new(shape.shape)
var _shape_remove: Callable = func(node: Node) -> void: ##removes a CollisionShape3D from the AABB cache and disconnects it's changed signal
	if node is CollisionShape3D:
		var s: Shape = _shapes.get(node)
		if s: s.disconnect_shape()
		_shapes.erase(node)

var _paths: Dictionary[Node3D, Path] ##all of the path states requested by navigation agents (any Node3D that calls next_point)
#NOTE: originally I was going to do _paths.set.bind(agent, state) and _paths.erase.bind(agent)
#      directly in the signals that use them, but that doesnt work because for connecting and
#      disconnecting they need to be the same Callable and also Dictionary doesnt nativley support
#      set and erase as Callables for signals to use I think
var _path_add: Callable = func(agent: Node3D, state: Path) -> void: _paths.set(agent, state) ##adds a path to the path cache paired with the node that requested it
var _path_remove: Callable = func(agent: Node3D) -> void: _paths.erase(agent) ##removes a path from the patch cache by agent
var _ver: int = 0 ##number used to update pathes- the verison changes when points are enabled/disabled, signaling pathes to recompute

var _mmp: MultiMeshInstance3D: ##the mesh displaying the points on the AStar3D grid
	get(): #ensures that there is always a valid instance- setting up and adding the mesh to the Complex
		if !is_instance_valid(_mmp):
			_mmp = MultiMeshInstance3D.new()
			var mm: MultiMesh = MultiMesh.new()
			mm.transform_format = MultiMesh.TRANSFORM_3D
			mm.use_colors = true
			mm.mesh = POINT_MESH
			_mmp.multimesh = mm
			add_child(_mmp)
		return _mmp
var _mc: MeshInstance3D: ##the mesh displaying path lines
	get(): #ensures that there is always a valid instance- setting up and adding the mesh to the Complex
		if !is_instance_valid(_mc):
			_mc = MeshInstance3D.new()
			_mc.material_override = CONNECTION_MATERIAL
			add_child(_mc)
		return _mc
var _timer: Timer: ##the timer used to keep track of time inbetween refreshed
	get(): #ensures that there is always a valid instance- setting up and adding the timer to the Complex
		if !is_instance_valid(_timer):
			_timer = Timer.new()
			_timer.wait_time = REFRESH_DELAY
			_timer.timeout.connect(_refresh)
			add_child(_timer)
			_timer.start()
		return _timer

func _init() -> void:
	tree_entered.connect(func() -> void: #when the Complex is added into the scene tree,
		print("starting refresh timer ", _timer) #start the timer
		get_tree().node_added.connect(_shape_add) #add/remove shapes when any node in the tree is added/removed
		get_tree().node_removed.connect(_shape_remove)
		for node: Node in find_children("*", "", true, false): #add current children of the Complex that are shapes
			_shape_add.call(node)
	)
	tree_exiting.connect(func() -> void: #when the Complex is removed from the scene tree,
		get_tree().node_added.disconnect(_shape_add) #disconnect the signals added when the Complex entered the tree
		get_tree().node_removed.disconnect(_shape_remove)
		_reset()
	)

##handles the obstacles and renders the grid
func _refresh() -> void:
	handle_obstacles()
	render_grid()

##resets the Complex, clearing all the caches and removing the dependent nodes
func _reset() -> void:
	_disabled = {}
	for s: Shape in _shapes.values(): s.disconnect_shape()
	_shapes = {}
	_paths = {}
	_ver = 0
	_mmp.queue_free()
	_mc.queue_free()
	_timer.queue_free()
	_refresh()

##returns a normalized position on the grid based on any position local to the grid by removing
##the spacing component and clamping it to the grid bounds
func _grid_point(p: Vector3) -> Vector3i:
	return Vector3i((p / spacing).round()).clamp(Vector3i.ZERO, Vector3i.ONE * (size - 1))

##returns a unique ID based on the normalized position of a point in the grid- basically a
##base-[size] number system where each place represents a axis calculated from the size of the grid.
##allows me to get points and make ids without having to run expensive astar searching functions.
func _point_id(x: int, y: int, z: int) -> int: return (x * size * size) + (y * size) + z

##loads a flattened 3D array of points onto the AStar3D grid with side lengths of size and spaced
##spacing meters apart for navigation
func generate_grid() -> void:
	#clears the grid and reserves space for the grid (saves memory since the size is defined)
	_astar.clear()
	_astar.reserve_space(size * size * size)
	for x: int in range(size):
		for y: int in range(size):
			for z: int in range(size):
				var i: int = _point_id(x, y , z) #get the id for the point using the current x/y/z values
				_astar.add_point(i, Vector3(x, y, z) * spacing) #add the point to the grid
				#in order for the algorithm to go between points, they have to be connected. the current
				#point is connected to the previous point on each of it's 3 axises if it's not the first
				#for that axis (though there will look like there's 6 connections on each inner point
				#in the end, the next point on the axis hasnt been created yet so we cannot connect it)
				#resulting in all points connected to their neighbors in all 6 directions
				if x > 0: _astar.connect_points(i, _point_id(x - 1, y, z))
				if y > 0: _astar.connect_points(i, _point_id(x, y - 1, z))
				if z > 0: _astar.connect_points(i, _point_id(x, y, z - 1))

##tracks all of the registered CollisionShape3Ds and disables the points that are inside of their
##shapes, only checking points that were disabled or are in one of their AABBs to improve performance
func handle_obstacles() -> void:
	var disabled: Dictionary[int, bool] #the new list of disabled points
	#physics stuff for the point querying that is reused
	var space: PhysicsDirectSpaceState3D = get_viewport().find_world_3d().direct_space_state
	var query: PhysicsPointQueryParameters3D = PhysicsPointQueryParameters3D.new()
	for shape: CollisionShape3D in _shapes:
		var cache: Shape = _shapes[shape].refresh(shape.shape, shape.global_transform) #gets the refreshed state of the shape
		#the start point and end points of the global AABB relative to the grid without spacing
		var a: Vector3i = _grid_point(cache.global.position) 
		var b: Vector3i = _grid_point(cache.global.end)
		#iterates through all of the points invetween the start/end points in the AABB. b+1 is used
		#so the points on the last faces are included.
		for x: int in range(a.x, b.x + 1):
			for y: int in range(a.y, b.y + 1):
				for z: int in range(a.z, b.z + 1):
					var i: int = _point_id(x, y, z) #converts the position to a point ID
					if disabled.has(i): continue #skip points that were disabled on this pass (not global _disabled)
					#if not disabled, but the point is indie a CollisionShape3D (if the query result 
					# isnt empty then it hit something, dont really care what it is tho), disable 
					#it in the local disabled points cache
					query.position = Vector3(x, y, z) * spacing
					if !space.intersect_point(query, 1).is_empty():
						disabled[i] = true
	for i: int in disabled: #all the points that were disabled in this pass
		if !_disabled.has(i): #if it's newly disabled, make it disabled in astar and update the version
			_astar.set_point_disabled(i, true)
			_ver += 1
	for i: int in _disabled: #all of the points that were disabled in the last pass
		if !disabled.has(i): #if it's not disabled anymore in this pass, re enable it in astar and update the version
			_astar.set_point_disabled(i, false)
			_ver += 1
	_disabled = disabled #make the local cache the global cache

##renders the AStar3D grid, disabled points, and currently tracked pathes for debug visualization
func render_grid() -> void:
	var path: Dictionary[int, bool] = {} #all points that are on a path- again Dictionarys are faster than Arrays
	var im: ImmediateMesh = ImmediateMesh.new() #mesh rendering the lines for the pathes
	#clear the mesh and set to using lines- every pair of points (2 consecutive , but no overlapping)
	#have a line drawn inbetween them 
	im.clear_surfaces()
	im.surface_begin(Mesh.PRIMITIVE_LINES)
	for p: Path in _paths.values(): #for each path, 
		if p.ids.is_empty(): continue #skip empty ones
		#for each point in the path incuding the first point- this is the one that the agent is on 
		#and isnt going towards, hence the -1. also clamped to the range of ids so no errors occur.
		for i: int in range(clampi(p.idx - 1, 0, p.ids.size() - 1), p.ids.size()):
			path[p.ids[i]] = true #add the point to be rendered differently in the MultiMesh pass
			if i < p.ids.size() - 1: #prevents i + 1 from causing an error
				#create the line segment between the current point and the next point
				im.surface_add_vertex(_astar.get_point_position(p.ids[i]))
				im.surface_add_vertex(_astar.get_point_position(p.ids[i + 1]))
	#complete the mesh and add it to the node rendering it in the scene
	im.surface_end()
	_mc.mesh = im
	#get the point ids from the AStar3D and the MultiMesh from the node
	var ids: PackedInt64Array = _astar.get_point_ids()
	var mm: MultiMesh = _mmp.multimesh
	mm.instance_count = ids.size() #number of instances, points rendered, is the number of points in the AStar3D
	#iterate thru a range of numbers. range() is not used because that makes a new Array, which is
	#expensive. the actual IDs arent iterated through either because the MultiMesh needs an index for
	#each of the instances, points, that it renders.
	for idx: int in ids.size():
		var i: int = ids[idx] #get the point ID
		#set the instance's position at index idx to the position on the AStar3D grid. because there's
		#no set_instance_position() function, we make a Transform3D using a default basis (positioned
		#at 0, no rotation, and scale of x1) and move it's origin (position) to the position of i.
		mm.set_instance_transform(idx, Transform3D(Basis(), _astar.get_point_position(i)))
		#sets the color of the points based on of their disabled, part of a path, or just idle
		mm.set_instance_color(idx, COLOR_DISABLED if _astar.is_point_disabled(i)
			else (COLOR_PATH if path.has(i) else COLOR_DEFAULT))

##called by agents, this gets the next point the agent should go to to reach the target position
func next_point(agent: Node3D, target: Vector3) -> Vector3:
	var p: Path = _paths.get(agent) #gets the path cached for the agent if it exists
	if p == null: #if it doesnt, make a new one
		p = Path.new()
		_paths[agent] = p
		#connect the signals to add the path to the cache when the agent enters the tree and
		#remove it when the agent exists the tree to prevent stale references
		var add: Callable = _path_add.bind(agent, p)
		var remove: Callable = _path_remove.bind(agent)
		if !agent.tree_entered.is_connected(add): agent.tree_entered.connect(add)
		if !agent.tree_exiting.is_connected(remove): agent.tree_exiting.connect(remove)
	#if there are no points in the path, the version of the Complex changed, the cached target is
	#different from the latest target queued for, or the position of the agent moved past the
	#deviation threshold from the point that the agent is supposed to be going torwards. the path
	#isnt updated when the agent is moving through it normaly to save memory
	if (p.ids.is_empty() or p.ver != _ver or !p.target.is_equal_approx(target)
	or (p.idx < p.ids.size() and agent.global_position.distance_to(_astar.get_point_position(p.ids[p.idx])) > spacing * DEVIATION_THRESHOLD)):
		#update the target position, calculate a new path from the agent to the target, reset the
		#indexing (the agent is now at the beginning of the path) and update the version
		p.target = target
		p.ids = _astar.get_id_path(_astar.get_closest_point(agent.global_position), _astar.get_closest_point(target))
		p.idx = 0
		p.ver = _ver
	if p.ids.is_empty(): return agent.global_position #if there's still no path, just return the position of the agent so it stays still
	if p.idx >= p.ids.size() - 1: return target #if the next target is past or at the last point, the target, return it
	#if not an edge case, get the next point position for the agent to go to- if the agent is close
	#enough to the point to advance, update the next point position to the next one on the path
	var next: Vector3 = _astar.get_point_position(p.ids[p.idx])
	if agent.global_position.distance_to(next) <= ADVANCE_THRESHOLD:
		p.idx += 1
		next = _astar.get_point_position(p.ids[p.idx])
	return next #return this next position to go to

##class storing a path state for each agent
class Path extends RefCounted:
	var target: Vector3 ##the last target position requested
	var ids: PackedInt64Array ##the points, in order, to get from the agent position to the target position
	var idx: int = 0 ##the current point the agent must go to next
	var ver: int = -1 ##the last recorded version of the Complex the path is running on

##class storing a shape state for a CollisionShape3D
class Shape extends RefCounted:
	var shape: Shape3D ##the raw shape resource of the CollisionShape3D
	var local: AABB ##the AABB of the shape local to the CollisionShape3D
	var trans: Transform3D ##the last saved transform of the CollisionShape3D
	var global: AABB ##the local AABB converted into global space via transformation
	var dirty: bool ##if shape changed (resource or values), requiring a refresh on the next get
	
	func _init(_shape: Shape3D) -> void: _update(_shape) #updates on initization
	
	##swaps the signal of the old shape with the new one if reconnect is true (2 functionalities 
	##merged into 1 call to reduce function calls), gets the AABB of it, and requests a refresh to be done
	func _update(_shape: Shape3D, reconnect: bool = true) -> void: 
		if reconnect: 
			disconnect_shape()
			shape = _shape
			shape.changed.connect(_update.bind(shape, false))
		local = shape.get_debug_mesh().get_aabb()
		dirty = true
	
	##updates the shape/transformation caches if the new values passed in are different, calculates
	##the global AABB, and returns the new Shape state.
	func refresh(_shape: Shape3D, _trans: Transform3D) -> Shape:
		if _shape != shape: _update(_shape)
		if dirty or _trans != trans:
			trans = _trans
			global = local * trans.inverse()
			dirty = false #changed have been accounted for so dirty can be cleared
		return self
	
	##disconnects the updated function from the changed signal of the shape
	func disconnect_shape() -> void:
		if is_instance_valid(shape) and shape.changed.is_connected(_update.bind(shape, false)):
			shape.changed.disconnect(_update.bind(shape, false))

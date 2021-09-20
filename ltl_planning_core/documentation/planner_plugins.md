# LTL Planner Plugins

The planner offers an interface to run python plugins.

## Launching plugins
To launch a plugin, add the plugin in the "/plugin" parameter of the planner node.
* Package and source file must be defined in "~plugin/<plugin-name>/path" on the format "package.source_file"
* Additional argument needed by the plugin can be defined on "~pluging/<plugin-name>/args/<argument>"

Example setup for the trap detection (needed for HIL mix initiative control):
```XML
<include file="$(find ltl_automaton_planner)/launch/ltl_planner.launch">
    <arg name="initial_ts_state_from_agent" default="False"/>
</include>

<group ns="ltl_planner/plugin">
    <group ns="TrapDetectionPlugin">
        <param name="path" value="ltl_automaton_hil_mic.trap_detection"/>
        <param name="args" value=""/>
    </group>
</group>

```

## Plugin specifications
### Naming
Name of the plugin should be the name of the class implementing it.

### __init__
The init method should take as input the ltl_planner object reference and an argument dictionary even if no argument is needed.

```Python
class PluginClass(object):
    def __init__(self, ltl_planner, args_dict):
```

### API methods
The plugins need to have the following API methods. If yhe method is not needed leave it empty:
* *init()*: Called once at initialization
* *set_sub_and_pub()*: Called after initialization to setup ROS subscribers, publishers, service clients & servers, action servers.
* *run_at_ts_update(ts_state)*: Called everytime a new TS state is received from the agent

### Accessing LTL planner elements
The plugin will be run from the planner.py node upper layer. From within the plugin code, use to access:
* ltl_planner: 
```python
self.ltl_planner
```
* Possible LTL states:
```python
self.ltl_planner.pssible_states
```
* Current ts state:
```python
self.ltl_planner.curr_ts_state
```
* Product graph:
```python
self.ltl_planner.product
```
* TS graph:
```python
self.ltl_planner.product.graph['ts']
```
* Büchi graph:
```python
self.ltl_planner.product.graph['buchi']
```
# ubtna_mobile_description package

### xacro

To convert the xacro file into a URDF file:
```bash
$ roscd ubtna_mobile_description/urdf/
$ rosrun xacro xacro --inorder iserve_trl7.xacro > iserve_trl7.urdf
```

### URDF

To check whether the sintax is fine or whether it have errors:
```bash
$ check_urdf iserve_trl7.urdf
```

To get the Graphviz in pdf:
```bash
$ sudo apt-get install graphviz
$ urdf_to_graphviz iserve_trl7.urdf
```

### Test

To run rviz test:
```bash
$ roslaunch ubtna_mobile_description ubtna_mobile_rviz.launch
```

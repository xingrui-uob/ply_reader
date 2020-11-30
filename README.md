PLY Reader
===

Extracting depth/colour images from PLY files

## Dependency

+ Glut: ```sudo apt install freeglut3-dev```
+ PCL with ```simulation``` module enabled (disabled by default)
+ Boost program_options module: ```sudo apt install libboost-program-options-dev```

## Usage

```bash
read_ply \
    --ply-path=${PLY_FILE} \
    --pose-file=${POSE_FILE} \
    --intr-file=${INTR_FILE} \
    --out-path=${PATH}
```
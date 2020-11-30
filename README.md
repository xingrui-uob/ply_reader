## Dependency

+ CMake
+ ```glut``` installed with ```apt install freeglut3-dev```
+ PCL with ```simulation``` module enabled (disabled by default)

## Usage

```bash
read_ply \
    --ply-path=${PLY_FILE} \
    --pose-file=${POSE_FILE} \
    --intr-file=${INTR_FILE} \
    --out-path=${PATH}
```
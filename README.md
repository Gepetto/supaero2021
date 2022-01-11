# Supaero robotics, 2022

This repository contains the exercices for the robotics class at Supaero, 2022.
The exercices are organized by notebook. Each notebook corresponds to one chapter of the class.
The notebooks are in Python and based on the software [Pinocchio](https://github.com/stack-of-tasks/pinocchio).

## Set up

### Linux, Python 3, PyPI

On a Linux system with Python 3, you can get the dependencies directly with:

```bash
python3 -m pip install -r requirements.txt
```

NB: you should consider using a [virtualenv](https://docs.python.org/3/library/venv.html)

Once you have the dependencies, you can start the server with `jupyter notebook`

### Docker

On other systems, a Docker image is provided, and can be started with:

```bash
docker run --net host gepetto/supaero
```

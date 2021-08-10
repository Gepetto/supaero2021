FROM memmos.laas.fr:5000/gepetto/buildfarm/robotpkg:20.04

RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    apt-get update -qqy && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    python3-pip \
    robotpkg-py38-example-robot-data \
 && python -m pip install --no-cache-dir \
    meshcat

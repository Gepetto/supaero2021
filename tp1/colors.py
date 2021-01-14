import meshcat

def rgb2int(r,g,b):
    return r*256**2 + g*256 + b


def material( color, transparent=False):
    mat = meshcat.geometry.MeshPhongMaterial()
    mat.color = color
    mat.transparent = transparent
    return mat

red     = material(color=rgb2int(255,  0,  0),transparent=False)
blue    = material(color=rgb2int(  0,  0,255),transparent=False)
green   = material(color=rgb2int(  0,255,  0),transparent=False)
yellow  = material(color=rgb2int(255,255,  0),transparent=False)
magenta = material(color=rgb2int(255,  0,255),transparent=False)
cyan    = material(color=rgb2int(  0,255,255),transparent=False)
white   = material(color=rgb2int(250,250,250),transparent=False)
black   = material(color=rgb2int(  5,  5,  5),transparent=False)
grey    = material(color=rgb2int(120,120,120),transparent=False)



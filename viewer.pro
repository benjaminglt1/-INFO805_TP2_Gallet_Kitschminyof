# Ceci est un fichier de configuration pour une application Qt
# Il faut peut-etre legerement l adapter pour votre ordinateur.

# nom de votre executable
TARGET  = ray-tracer
# config de l executable
CONFIG *= qt opengl release
CONFIG += c++11
# config de Qt
QT     *= opengl xml
QMAKE_CXXFLAGS += -std=c++11

# Noms de vos fichiers entete
HEADERS = Viewer.h PointVector.h Color.h Sphere.h GraphicalObject.h Light.h \
          Material.h PointLight.h Image2D.h Image2DWriter.h Renderer.h Ray.h \
          Scene.h

# Noms de vos fichiers source
SOURCES = Viewer.cpp ray-tracer.cpp Sphere.cpp


###########################################################
# Commentez/decommentez selon votre config/systeme
# (Une config windows est possible)
###########################################################

# Exemple de configuration Windows :
LIBS *= -lopengl32 -lglu32
INCLUDEPATH *= D:\Logiciel\QtLib\libQGLViewer-2.7.2\libQGLViewer-2.7.2
LIBS *= -LD:\Logiciel\QtLib\libQGLViewer-2.7.2\libQGLViewer-2.7.2\QGLViewer -lQGLViewer2

DISTFILES += \
    .gitignore \
    README.md \
    tp-ig-2.dox \
    tref.tri

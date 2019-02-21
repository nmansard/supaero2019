# in crocoddyl
git submodule update --init

# install classic
sudo apt install libeigen3-dev ffmpeg cmake
pip install --user --upgrade pip
pip install --user --upgrade matplotlib
pip install --user --upgrade scipy

# Install pinocchio
cd ~
git clone https://github.com/stack-of-tasks/pinocchio --recursive
cd pinocchio
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/home/student/compil -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_UNIT_TESTS=OFF
make -sj4 install

# Set up new pinocchio
export PYTHONPATH=/home/student/compil/lib/python2.7/dist-packages:${PYTHONPATH}
echo "export PYTHONPATH=/home/student/compil/lib/python2.7/dist-packages:${PYTHONPATH}" >> ~/.bashrc


                                                                          


sudo apt install curl aptitude git adb
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub xenial robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt-get update
sudo apt install robotpkg-py27-pinocchio robotpkg-gepetto-viewer-corba ipython ipython3 python-pip python3-pip freeglut3 python-matplotlib python3-matplotlib
pip3 install --user 'cozmo[camera,3dviewer]'
pip3 install --user tflearn tensorflow
pip install --user tflearn tensorflow
pip3 install --user ipython


echo '
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH
' >> ~/.bashrc
source ~/.bashrc

export WITH_PYOMO=

if [ ! -z $WITH_PYOMO ] ; then
  cd /tmp   
  wget https://repo.anaconda.com/archive/Anaconda3-5.3.1-Linux-x86_64.sh
  sh Anaconda3-5.3.1-Linux-x86_64.sh
  conda install -c conda-forge pyomo
  #conda install -c conda-forge pyomo.extra
  conda install -c conda-forge ipopt
fi 

'
=== Installing Cozmo SDK on your phone and your PC

*** On the phone.
1. Install cozmo
2. Connect to Cozmo wireless
3. Plug the phone to computer by USB
4. Enable the USB debuging https://www.kingoapp.com/root-tutorials/how-to-enable-usb-debugging-mode-on-android.htm
5. Activate debuging (in phone parameter, developer options, activate)
   - activate the main switch
   - activate "debug" mode
6. Check USB debuging in terminal by asking adb
   (cozmo) nmansard@koyasan:cozmo [18:22] adb devices
   List of devices attached 
   ZY22366T55	device
7. In the cozmo app, go to connect, then parameters (top-right icon) >> activate SDK

*** On the computer.
1. http://cozmosdk.anki.com/docs/install-linux.html
Everything is already set up using the command lines above.
2. Run a test
   python3 cozmo_example.py
   

'

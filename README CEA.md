# TOURELLE_ROS

## Dépendances
Installer la stack Dynamixel de ROS :

        $ sudo apt-get install ros-indigo-dynamixel-motor

Installer le paquet `usb_cam` :

        $ sudo apt-get install ros-indigo-usb-cam

Remplacez indigo par hydro selon la version.

## Gestion Dynamixel
La stack du wiki ROS](http://wiki.ros.org/dynamixel_motor) est maintenant utilisée pour faciliter l'interfaçage avec Gazebo.

### Compatibilite du package ROS dynamixel_motor avec l'adaptateur USB2AX
L'utilisation de l'adaptateur USB2AX avec dynamixel_motor sous Ubuntu néccessite de patcher le fichier dynamixel_io.py du package comme ceci:

	cd /opt/ros/indigo/lib/python2.7/dist-packages/dynamixel_driver/
	sudo scite dynamixel_io.py

	class DynamixelIO(object):
	""" Provides low level IO with the Dynamixel servos through pyserial. Has the
	ability to write instruction packets, request and read register value
	packets, send and receive a response to a ping packet, and send a SYNC WRITE
	multi-servo instruction packet.
	"""

    def __init__(self, port, baudrate):
        """ Constructor takes serial port and baudrate as arguments. """
        try:
            self.serial_mutex = Lock()
            self.ser = None
    """ # A commenter
            self.ser = serial.Serial(port)
	        self.ser.setTimeout(0.015)
            self.ser.baudrate = baudrate
            self.port_name = port
    """
    #A rajouter
	    self.ser = serial.Serial(port,baudrate,timeout=0.04)
	    
        except SerialOpenError:
           raise SerialOpenError(port, baudrate)

### Topic
Chaque servomoteur a deux topics, un de commande `command` et un d'état `state`.

- `/command` sert à  envoyer des commandes sous formes de messages de type `std_msgs/Float64` pour donner l'angle en radians;
- `/state` sert à  donner l'état du servo sous la forme de messages de type `dynamixel_msgs/JointState` :

        Header header
        string name         # joint name
        int32[] motor_ids   # motor ids controlling this joint
        int32[] motor_temps # motor temperatures, same order as motor_ids

        float64 goal_pos    # commanded position (in radians)
        float64 current_pos # current joint position (in radians)
        float64 error       # error between commanded and current positions (in radians)
        float64 velocity    # current joint speed (in radians per second)
        float64 load        # current load
        bool is_moving      # is joint currently in motion

Ici on a donc les topics suivants :

        /tourelle/j_BASE_ACC_position_controller/command
        /tourelle/j_BASE_ACC_position_controller/state
        /tourelle/j_ACC_PLT_position_controller/command
        /tourelle/j_ACC_PLT_position_controller/state

Pour publier une commande manuellement :

        $ rostopic pub -1 /tourelle/j_ACC_PLT_position_controller/command std_msgs/Float64 -- 1.0

### Services
Différents services sont disponibles sur chaque contrôleur :

    RestartController
    SetComplianceMargin
    SetCompliancePunch
    SetComplianceSlope
    SetSpeed
    SetTorqueLimit
    StartController
    StopController
    TorqueEnable

Exemple, pour désactiver le couple d'un moteur :

        $ rosservice call /tourelle/j_BASE_ACC_position_controller/torque_enable False

## PCDUINO
pour se connecter en ssh : `ssh ubuntu@132.169.195.246` pw :`ubuntu`

Au demarrage, pensez à  mettre en jour l'heure :     
     - soit en manuel mais pas précis :`sudo date MMDDhhmmYYYY`
     - soit en ssh depuis le pc duino :`sudo date --set="$(ssh darwin@darwin.local 'date -u')"`
Si probleme de fuseau horaire : `sudo dpkg-reconfigure tzdata`

Si connexion internet, et pas de parfeux, il y a d'autres solution, `chrony` ou `ntpdate`

Pour le reglage réseau
    - se mettre ne DHCP `Dhcp_M@r!`
    - parametrez `ROS_HOSTNAME` et `ROS_MASTER_URI` dans le `.bashrc` ou `rosbash.bash`

Faire un `devel` dans le workspace si vous avez le `rosbash` generation robots, sinon sourcez le.
Avec un master qui tourne sur le PC fixe, il suffit ensuite de lancer le launch

     roslaunch tourelle_ros tourelle-pcduino.launch

3 noeuds sont alors lancées, `/telemeter`, `/usb_cam_node` et `/tourelle/dynamixel_manager` et les topics correspondants publiés

pour copier sur le pcduino en ssh, se mettre à la racine du dossier sur le pc fixe et :
    
    scp -rp tourelle_ros ubuntu@132.169.195.224:/home/ubuntu/tourelle-ws/src
pour copier que les fichier modifiés, a partir du pcduino :
	
    cd ~/tourelle-ws/src/tourelle_ros`
	rsync -vazy -e ssh darwin@132.169.194.214:/home/darwin/Jerem-ws/src/tourelle_ros/ ./`

### Changer message d'accueil
Il s'agit du message informatif afficher au début d'une connexion SSh par exemple.

Créer le fichier `/etc/motd` et y inscrire le message souhaité.

Exemple :
#########################################
	* Penser a regler la date depuis le pc embarque :
		sudo date --set="$(ssh darwin@darwin.local 'date -u')"
	* Pour copier en ssh a partir du pc embarque :
		cd ~/tourelle-ws/src/tourelle_ros
		rsync -vazy -e ssh darwin@darwin.local:/home/darwin/Jerem-ws/src/tourelle_ros/ ./
#########################################

et mettre `no` en face de `PrintLastLog` dans `/etc/ssh/sshd_config`

Source : <http://serverfault.com/questions/407033/how-can-i-edit-the-welcome-message-when-ssh-start>

## ODROID XU4
probleme avec robot state publisher:

	cd ~/Downloads
	wget http://launchpadlibrarian.net/182261128/libpcre3_8.35-3ubuntu1_armhf.deb
	wget http://launchpadlibrarian.net/182261132/libpcre3-dev_8.35-3ubuntu1_armhf.deb
	wget http://launchpadlibrarian.net/182261135/libpcre3-dbg_8.35-3ubuntu1_armhf.deb
	wget http://launchpadlibrarian.net/182261131/libpcrecpp0_8.35-3ubuntu1_armhf.deb
	sudo dpkg -i libpcre*8.35*.deb

source :<http://answers.ros.org/question/209686/turtlebot-apt-get-update-has-generated-an-error-on-minimallaunch/>

mauvais fuseau horaire :
modifier `/etc/environment`, et ajoutez la variable `TZ="Europe/Paris"` et redemarrer
source : <http://doc.ubuntu-fr.org/heure_systeme>

## PCFIXE

    roscore
Aprés le launch sur le PC embarqué :

    roslaunch tourelle_ros tourelle_pcfixe.launch

### installation wifi usb RT2800/2870
<http://www.cyberciti.biz/tips/linux-install-rt2870-chipset-based-usb-wireless-adapter.html>

    wget http://distfiles.lesslinux.org/2010_0709_RT2870_Linux_STA_v2.4.0.1.tar.bz2`
<http://learn.linksprite.com/pcduino/pcduino-embedded-linux-development/install-pcduino-linux-headers-3-4-79-and-compile-driver-on-pcduino/>
    
    sudo apt-get install pcduino-linux-headers-3.4.79+

## Wiimote bluetooth
<http://www.raspberrypi-spy.co.uk/2013/02/nintendo-wii-remote-python-and-the-raspberry-pi/>
<http://linux.arcticdesign.fr/commande-bluetooth-laide-du-pyhton/>

pour activer le bluetooth sur le pc H@ri, a chaque connexion du dongle :
    
    sudo hciconfig hci0 reset

## CONFIGURATION SONDE GR
### Dependances

    sudo apt-get install python-dev
    sudo apt-get install python-tz
    sudo apt-get install libudev-dev

    wget http://pypi.python.org/packages/source/C/Cython/Cython-0.16.tar.gz
    tar xzf Cython-0.16.tar.gz
    cd Cython-0.16
    sudo python setup.py install

    wget http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.9/libusb-1.0.9.tar.bz2
    tar xjf libusb-1.0.9.tar.bz2
    cd libusb-1.0.9
    ./configure
    make
    sudo make install

### Hid Ubuntu 12.04 :

    git clone https://github.com/trezor/cython-hidapi.git
    cd cython-hidapi
    git submodule init
    git submodule update

Ouvrir le fichier setup.py puis Remplacer le chemin  `/usr/include/libusb-1.0`  par `/usr/local/include/libusb-1.0` Poursuivre ensuite en tapant la commande :
 
    sudo python setup.py install

### Hid Ubuntu 14.04 :

    git clone https://github.com/gbishop/cython-hidapi.git
    cd cython-hidapi
    python setup.py build

Ouvrir le fichier setup.py puis mettre :
        
        from distutils.core import setup
        from distutils.extension import Extension
        from Cython.Distutils import build_ext
        import os
        os.environ['CFLAGS'] = "-I/usr/local/include/libusb-1.0"
        os.environ['LDFLAGS'] = "-L/usr/lib/i386-linux-gnu -lusb-1.0 -ludev -lrt" 
        setup(
            cmdclass = {'build_ext': build_ext},
            ext_modules = [Extension("hid", ["hid.pyx", "hid-libusb.c"],
                          libraries=["usb-1.0", "udev", "rt"])]
        )
 
    sudo python setup.py install

### Accés

    & udevadm info -a -n hidraw0

    ...
    looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-1':
    KERNELS=="1-1"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ATTRS{bDeviceSubClass}=="00"
    ATTRS{bDeviceProtocol}=="00"
    ATTRS{devpath}=="1"
    ATTRS{idVendor}=="2a5a"
    ATTRS{speed}=="12"
    ATTRS{bNumInterfaces}==" 1"
    ATTRS{bConfigurationValue}=="1"
    ATTRS{bMaxPacketSize0}=="64"
    ATTRS{busnum}=="1"
    ATTRS{devnum}=="7"
    ATTRS{configuration}==""
    ATTRS{bMaxPower}=="64mA"
    ATTRS{authorized}=="1"
    ATTRS{bmAttributes}=="80"
    ATTRS{bNumConfigurations}=="1"
    ATTRS{maxchild}=="0"
    ATTRS{bcdDevice}=="0212"
    ATTRS{avoid_reset_quirk}=="0"
    ATTRS{quirks}=="0x0"
    ATTRS{serial}=="150004"
    ATTRS{version}==" 1.10"
    ATTRS{urbnum}=="17"
    ATTRS{ltm_capable}=="no"
    ATTRS{manufacturer}=="Kromek"
    ATTRS{removable}=="unknown"
    ATTRS{idProduct}=="0050"
    ATTRS{bDeviceClass}=="00"
    ATTRS{product}=="GR05"
    ...


Il faut ensuite créer un fichier d'autorisation d'accés  au périphérique. Pour cela se placer dans le répertoire `rules.d` puis créer un fichier `GR1.rules` en tant que super utilisateur :
 
    cd /etc/udev/rules.d
    sudo scite GR1.rules
 
Dans ce fichier, noterla ligne suivante :
 
    SUBSYSTEMS=="usb", ATTRS{idVendor}=="2a5a",ATTRS{idProduct}=="0050",MODE="0666"

## Camera
La camera utilisée sur la maquette est une Microsoft Corp. LifeCam Studio (ID 045e:0772)

### Package ROS : usb_cam
Nous utilisons le package ROS usb_cam : http://wiki.ros.org/usb_cam
Ce package s'appuie sur le drive camera UVC (USB Video Class) : http://www.ideasonboard.org/uvc/

### Calibration de la camera
Une calibration de la camera est nécessaire au fonctionnement du noeud image_ar afin de pouvoir projeter les points 3D -> 2D. Cela permet également une correction des distorsions de l'image.
Suivre le tuto: http://wiki.ros.org/camera_calibration

### Problème d'instabilité
Vérifier la compatibilité de la camera avec la librairie UVC (voir http://www.ideasonboard.org/uvc/).
Pour la cas de la LifeCam Studio la compatibilité n'est pas totale et des warning apparaissent lors du lancement du noeud usb_cam voir des problème d'instabilité.
Une piste pour remédier au problème de stabilité : https://help.ubuntu.com/community/UVC

Ajouter un fichier de configuration: /etc/modprobe.d/uvcvideo.conf

Editer le fichier avec

	...
	options uvcvideo nodrop=1
	...

Sauvegarder et recharger le driver:

	...
	sudo rmmod uvcvideo && sudo modprobe uvcvideo
	...

En test:
- sauts d'image constatés lors de l'application de cette solution
- stabilité un peu meilleure mais pas totale

## Plugin Rqt
<http://pyqwt.sourceforge.net/doc5/installation.html>
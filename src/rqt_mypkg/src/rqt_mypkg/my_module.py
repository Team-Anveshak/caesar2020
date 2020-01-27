import os
import rospy
import rospkg
import threading
from argparse import ArgumentParser
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from .my_widget import MyWidget

class MyPlugin(Plugin):

    def __init__(self, context):
        super(Plugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin') 
        # Process standalone plugin command-line arguments
        #from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

	
	

    
        self._widget = MyWidget(context)
	#ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        #loadUi(ui_file, self._widget)
	#self._widget.setObjectName('MyPluginUi')
	#self.textEdit_3.setText("done") 
	#self.test.clicked[bool].connect(self.a)
        #self.destroyed.connect(self.b)
	#print "hello"
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
	
    #def a(self):
	#print"hi"
    #def b(self):
	#print"bye"

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

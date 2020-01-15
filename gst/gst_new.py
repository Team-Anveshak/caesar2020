#! /usr/bin/env python
import gi
gi.require_version('Gtk', '3.0')
#gi.require_version('Gst','1.0')
#from gi.repository import Gst
from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import GObject
import sys, os, time, signal
import subprocess

class GTK_Main:

    def __init__(self):
	self.s1=1
	self.s2=2
        window = Gtk.Window(Gtk.WindowType.TOPLEVEL)
        window.set_default_size(600,200)
        window.connect("destroy", Gtk.main_quit, "WM destroy")
        hbox = Gtk.HBox()
        window.add(hbox)
	vbox1 = Gtk.VBox()
	hbox.add(vbox1)
	self.button1_1 = Gtk.Button("Source1")
	self.button1_1.connect("clicked", self.start_1_1)
        vbox1.add(self.button1_1)
	self.button2_1 = Gtk.Button("Source2")
	self.button2_1.connect("clicked", self.start_2_1)
        vbox1.add(self.button2_1)
	self.button3_1 = Gtk.Button("Source3")
	self.button3_1.connect("clicked", self.start_3_1)
        vbox1.add(self.button3_1)
	self.button4_1 = Gtk.Button("Source4")
	self.button4_1.connect("clicked", self.start_4_1)
        vbox1.add(self.button4_1)
	vbox2 = Gtk.VBox()
	hbox.add(vbox2)
	self.button1_2 = Gtk.Button("Source1")
	self.button1_2.connect("clicked", self.start_1_2)
        vbox2.add(self.button1_2)
	self.button2_2 = Gtk.Button("Source2")
	self.button2_2.connect("clicked", self.start_2_2)
        vbox2.add(self.button2_2)
	self.button3_2 = Gtk.Button("Source3")
	self.button3_2.connect("clicked", self.start_3_2)
        vbox2.add(self.button3_2)
	self.button4_2 = Gtk.Button("Source4")
	self.button4_2.connect("clicked", self.start_4_2)
        vbox2.add(self.button4_2)
        window.show_all()

    def start_1_1(self, w):
	if (self.s2!=1):
		self.s1=1
		self.run()

    def start_2_1(self, w):
	if (self.s2!=2):
		self.s1=2
		self.run()

    def start_3_1(self, w):
	if (self.s2!=3):
		self.s1=3
		self.run()

    def start_4_1(self, w):
	if (self.s2!=4):
		self.s1=4
		self.run()

    def start_1_2(self, w):
	if (self.s1!=1):
		self.s2=1
		self.run()

    def start_2_2(self, w):
	if (self.s1!=2):
		self.s2=2
		self.run()

    def start_3_2(self, w):
	if (self.s1!=3):
		self.s2=3
		self.run()

    def start_4_2(self, w):
	if (self.s1!=4):
		self.s2=4
		self.run()

    def run(self):
	f=open("run.sh","w")
	f.write("#!/bin/bash\ngst-launch-1.0 videomixer name=mix ! nvvidconv ! omxh265enc control-rate=2 bitrate=2000 ! 'video/x-h265, stream-format=(string)byte-stream' ! h265parse ! rtph265pay mtu=1400 ! udpsink host=192.168.0.4 port=5001 sync=false async=false v4l2src device='/dev/video%d' ! video/x-raw, width=640, height=480 ! videobox border-alpha=0 top=0 left=-640 ! mix. v4l2src device='/dev/video%d' ! video/x-raw, width=640, height=480 ! videobox border-alpha=0 top=0 left=0 ! mix." % ((self.s1-1), (self.s2-1))
	f.close()
	os.system("pkill gst-launch-1.0")			
	subprocess.Popen("./run.sh")

if __name__ == '__main__':
	GObject.threads_init()      
	GTK_Main() 
	Gtk.main()
	

#!/usr/bin/python

import math
import pygame,thread,sys
from thread import * 
import socket
import time

BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

class Run:
	def __init__(self):
		pygame.init()
		pygame.joystick.init()
		self.joystick=pygame.joystick.Joystick(0)
		pygame.joystick.init()
		host='192.168.0.2'
		port=10006
		self.s=socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
		self.s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
		try:
			self.s.bind((host,port))
			print("Waiting for connection...")
		except Exception as e:
			print(str(e))
		self.l1=0
		self.r1=0
		self.straight=0
		self.zero_turn=0
		self.d=1
		self.hb=0
		self.s_arr=[50,75,140,200,800]
		self.vel=0
		self.omega=0
		self.a=0
		self.b=0
		start_new_thread(self.joyEvent,())
	def joyEvent(self):
		try:
			self.joystick.init()
		except Exception:
			print("[ERROR] initialising joystick") ; sys.exit(0)
		while True:
			for event in pygame.event.get():
				pass
			self.l1=self.joystick.get_button(4)
			self.r1=self.joystick.get_button(5)
			self.hb=self.joystick.get_button(7)
			#self.straight=self.joystick.get_axis(1)
			#self.zero_turn=self.joystick.get_axis(2)
			if (self.r1 == 0):
				self.a=0
			if (self.r1==1) and (self.a==0):
				if self.d <5:
					self.d=self.d+1
					self.a=1
			if (self.l1 == 0):
				self.b=0
			if (self.l1==1) and (self.b==0):
				if self.d >1:
					self.d=self.d-1
					self.b=1						
	def spin(self):
		while True:
			data=self.s.recvfrom(1024)
			while True:
				self.straight=self.joystick.get_axis(1)
				self.zero_turn=self.joystick.get_axis(2)
				self.straight=self.straight*(-1)
				if (abs(self.straight)>0.05 or abs(self.zero_turn)>0.05):
					self.vel = self.straight *self.s_arr[self.d-1]
					self.omega =self.zero_turn *self.s_arr[self.d-1]
				else:
					self.vel=0
					self.omega=0
				self.vel=1024+self.vel
				self.omega=1024-self.omega
				try:
					if data:
						message=str(self.vel)+','+str(self.omega)+','+str(self.hb)+'.'
						self.s.sendto(message,data[1])
						print(message)
				except:
					break
				time.sleep(0.1)

if __name__=='__main__':
	run=Run()
	run.spin()

#!/bin/python

import wx
import sys
import time
import os
from subprocess import Popen, PIPE, STDOUT


class RedirectText:
	def __init__(self,aWxTextCtrl):
		self.out=aWxTextCtrl
	
	def write(self,string):
		self.out.WriteText(string)

class HelloFrame(wx.Frame):
	"""
	A Frame that says Hello World
	"""
	
	def __init__(self, *args, **kw):
		self.running = True
		
		# ensure the parent's __init__ is called
		super(HelloFrame, self).__init__(*args, **kw)
		
		# create a panel in the frame
		pnl = wx.Panel(self)
		
		# and put some text with a larger bold font on it
		st = wx.StaticText(pnl, label="Hello World!", pos=(25,25))
		font = st.GetFont()
		font.PointSize += 10
		#font = font.Bold()
		st.SetFont(font)
		
		# add a button
		test_button = wx.Button(pnl, id=wx.ID_ANY, label="Test Button", pos=(25,75))
		test_button.Bind(wx.EVT_BUTTON, self.onTestButton)
		
		# add a button
		test2_button = wx.Button(pnl, id=wx.ID_ANY, label="Test 2 Button", pos=(175,75))
		test2_button.Bind(wx.EVT_BUTTON, self.onTest2Button)
		
		
		# Test output box
		self.log = wx.TextCtrl(pnl, -1, size=(500,400),
					pos=(25,125),
					style = wx.TE_MULTILINE|
					wx.HSCROLL)
		self.redir=RedirectText(self.log)
		sys.stdout=self.redir
		
		
		# create a menu bar
		self.makeMenuBar()
		
		# and a status bar
		self.CreateStatusBar()
		self.SetStatusText("Welcome to wxPython!")

	def onTestButton(self, event):
		print("Button Pressed")
		
		
		#proc = Popen(['ping 4.2.2.2'], stdout=PIPE, stdin=open(os.devnull), stderr=STDOUT, shell=True)
		#proc = Popen(['rosrun evo_ros software_manager.py'], stdout=PIPE, stdin=open(os.devnull), stderr=STDOUT, shell=True)
		proc = Popen(['python test_transporter.py'], stdout=PIPE, stdin=open(os.devnull), shell=True)

		while self.running:
			try: line = proc.stdout.readline()
			except:
				wx.Yield()
				time.sleep(0.5)
				continue                       
			wx.Yield()
			if line.strip() == "":
				pass
			else:
				print line.strip()
		proc.terminate()
		proc.wait()
		
		
	def onTest2Button(self, event):
		print('Button 2 pressed')
		self.running = not self.running	
	
	def makeMenuBar(self):
		"""
		A menu bar is composed of menus, which are composed of menu items.
		This method builds a set of menus and binds handlers to be called
		when the menu item is selected.
		"""
		
		# Make a file menu with Hello and Exit items
		fileMenu = wx.Menu()
		# The "\t..." syntax defines an accelerator key that also triggers
		# the same event
		helloItem = fileMenu.Append(-1, "&Hello...\tCtrl-H",
				"Help string shown in status bar for this menu item")
		fileMenu.AppendSeparator()
		# When using a stock ID we don't need to specify the menu item's
		# label
		exitItem = fileMenu.Append(wx.ID_EXIT)
		
		# Now a help menu for the about item
		helpMenu = wx.Menu()
		aboutItem = helpMenu.Append(wx.ID_ABOUT)
		
		# Make the menu bar and add the two menus to it. The '&' defines
		# that the next letter is the "mnemonic" for the menu item. On the
		# platforms that support it those letters are underlined and can be
		# triggered from the keyboard.
		menuBar = wx.MenuBar()
		menuBar.Append(fileMenu, "&File")
		menuBar.Append(helpMenu, "&Help")
		
		# Give the menu bar to the frame
		self.SetMenuBar(menuBar)
		
		# Finally, associate a handler function with the EVT_MENU event for
		# each of the menu items. That means that when that menu item is
		# activated then the associated handler function will be called.
		self.Bind(wx.EVT_MENU, self.OnHello, helloItem)
		self.Bind(wx.EVT_MENU, self.OnExit,  exitItem)
		self.Bind(wx.EVT_MENU, self.OnAbout, aboutItem)
	

	def OnExit(self, event):
		"""Close the frame, terminating the application."""
		self.Close(True)

	
	def OnHello(self, event):
		"""Say hello to the user."""
		wx.MessageBox("Hello again from wxPython")

	
	def OnAbout(self, event):
		"""Display an About Dialog"""
		wx.MessageBox("This is a wxPython Hello World sample",
						"About Hello World 2",
						wx.OK|wx.ICON_INFORMATION)


if __name__ == '__main__':
    # When this module is run (not imported) then create the app, the
    # frame, show it, and start the event loop.
    app = wx.App()
    frm = HelloFrame(None, -1,
					size=(600,600),
					title='Hello World 2')
    frm.Show()
    app.MainLoop()

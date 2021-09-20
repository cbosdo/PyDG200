#! /usr/bin/env python3

#######################################
############## PyDG200 ################
#######################################
# Global Sat DG200 manager, for configuration setting and GPX tracks
# downloading.
#
# Version: 1.4
# changelog: 
#   -1.5 :
#       - bug correction for below 0 elevations
#   -1.4 :
#       - bug correction for separated header lists 
#   -1.3 :
#       - bug after clear memory corrected
#       - get_configuration after downloading to get the diode on
#       - bug correction for longitude/latitude < 0
#   -1.2 :
#       - bug correction for longitude/latitude < 1
#   -1.1 :
#       - switch to GPX 1.1
#       - years are counted from 2000
# Author: Aurelien Coillet, acoillet@free.fr
# Needs:
#   python 3
#   python3-pyserial
#   gtk+ 
# Licence: GPL3 http://gplv3.fsf.org/
#

import os
import time
from serial import Serial
from gi.repository import Gtk

isDebug = False

def int2bytes(number,nbBytes):
    if number>= 0:
        hexformat = '{0:0>' + str(nbBytes * 2) + 'X}'
        return ["0x" + hexformat.format(number)[2*(x-1):2*x] for x in range(1,nbBytes+1)]
    else:
        return None

def bytes2int(bytes):
    return int(''.join(bytes).replace('0x',''),16)

def process_point(raw_point):
    if isDebug:
        print(raw_point)

    if bytes2int(raw_point[0]) > 128: # then it's negative
        mega_latitude = '{0:0>9d}'.format(2**32 - bytes2int(raw_point[:4]))
        latitude = - float(str(mega_latitude)[:-6]) \
            - float(str(mega_latitude)[-6:])/600000
    else:
        mega_latitude = '{0:0>9d}'.format(bytes2int(raw_point[:4]))
        latitude = float(str(mega_latitude)[:-6]) \
            + float(str(mega_latitude)[-6:])/600000
    if bytes2int(raw_point[4]) > 128: # then it's negative
        mega_longitude = '{0:0>9d}'.format(2**32 - bytes2int(raw_point[4:8]))
        longitude = - float(str(mega_longitude)[:-6]) \
            - float(str(mega_longitude)[-6:])/600000
    else:
        mega_longitude = '{0:0>9d}'.format(bytes2int(raw_point[4:8]))
        longitude = float(str(mega_longitude)[:-6]) \
            + float(str(mega_longitude)[-6:])/600000
    utime = \
    '{0:0>6d}'.format(bytes2int(raw_point[9:12]))
    udate = \
    '{0:0>6d}'.format(bytes2int(raw_point[12:16]))
    timestamp = '20' + udate[4:6] + '-' + udate[2:4] + \
            '-' + udate[:2] + 'T' + utime[:2] + ':' + \
            utime[2:4] + ':' + utime[4:6] + 'Z'
    speed = bytes2int(raw_point[16:20]) / 100 / 3.6  # speed in m/s
    if len(raw_point) == 20:
        return [latitude, longitude, timestamp, speed]
    elif len(raw_point) == 32:
        if bytes2int(raw_point[20]) > 128: # then it's negative
            altitude = - (2**32 - bytes2int(raw_point[20:24])) / 10000
        else:
            altitude = bytes2int(raw_point[20:24]) / 10000
        return [latitude, longitude, timestamp, speed, altitude]
    else:
        return None

def write_gpx(folder, track):
    filename = folder + '/' + track[0][2] + '.gpx'
    gpx_file = open(filename, 'w')
    gpx_file.write('<?xml version="1.0" encoding="UTF-8" standalone="no" ?>\n')
    gpx_file.write('<gpx xmlns="http://www.topografix.com/GPX/1/1"\
 creator="Py3DG200" version="1.6"\
 xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"\
 xsi:schemaLocation="http://www.topografix.com/GPX/1/1\
 http://www.topografix.com/GPX/1/1/gpx.xsd">\n')

    for point in track:
        # first iteration to find waypoints
        if abs(point[0]) > 100:
            gpx_file.write('    <wpt lat="' + format((point[0]-100), '.7f') +\
                    '" lon="' + format(point[1], '.7f') + '">\n')
            gpx_file.write('      <time>' + point[2] + '</time>\n')
            gpx_file.write('      <speed>' + str(point[3]) + '</speed>\n')
            if len(point) == 5:
                gpx_file.write('      <ele>' + str(point[4]) + '</ele>\n')
            gpx_file.write('    </wpt>\n')

    gpx_file.write('  <trk>\n')
    gpx_file.write('    <name>DG-200 ' + track[0][2] + '</name>\n')
    gpx_file.write('    <trkseg>\n')
    for point in track:
        # Now for track points
        if abs(point[0]) < 100:
            gpx_file.write('      <trkpt lat="' + format(point[0], '.7f') + '" lon="' + \
                    format(point[1], '.7f') + '">\n')
            gpx_file.write('        <time>' + str(point[2]) + '</time>\n')
            gpx_file.write('        <speed>' + str(point[3]) + '</speed>\n')
            if len(point) == 5:
                gpx_file.write('        <ele>' + str(point[4]) + '</ele>\n')
            gpx_file.write('      </trkpt>\n')

    gpx_file.write('    </trkseg>\n')
    gpx_file.write('  </trk>\n')
    gpx_file.write('</gpx>\n')
    gpx_file.close()


class DG200(Serial):
    '''DG200 Class with appropriate methods for sending and receiving data from
    it'''
    def __init__(self):
        Serial.__init__(self)
        self.baudrate = 230400
        self.timeout = 1

    def connect(self,tty_USB):
        try:
            self.port = tty_USB
            self.open()
            if self.isOpen():
                if isDebug:
                    print('Connection to ' + tty_USB + ' successful!')
                return 1
            else:
                print("Warning: can't connect to " + tty_USB)
                return 0
        except:
            print("Warning: can't connect to " + tty_USB)
            return 0

    def send(self,payload):
        sequence = ['0xA0', '0xA2'] # start sequence
        if len(payload) <= int('ffff',16):
            # payload length (2 bytes)
            sequence.append("0x" + "{0:0>4X}".format(len(payload))[0:2]) 
            sequence.append("0x" + "{0:0>4X}".format(len(payload))[2:4])
        else:
            print('Warning: payload is too long, aborting')
            return 0
        sequence.extend(payload) # payload sequence
        checksum = 0 # calculating checksum
        for x in payload:
            checksum = checksum + int(x,16)
        checksum = checksum % (2**15-1)
        # append checksum (2 bytes)
        sequence.append("0x" + "{0:0>4X}".format(checksum)[0:2])
        sequence.append("0x" + "{0:0>4X}".format(checksum)[2:4])
        sequence.extend(['0xB0', '0xB3']) # end sequence

        if isDebug:
            print("Sent: " + str(sequence))
        seq = bytes.fromhex(''.join(sequence).replace('0x',''))
        bytes_transfered = self.write(seq)
        return(bytes_transfered)

    def receive(self):
        payload = ['0x12']
        check = 0
        while payload[0] == '0x12' or check == 1:
            header = self.read(4)
            if len(header) != 4:
                print(header)
                return None
            payload_length = header[2]*256+header[3]
            if isDebug:
                print('longueur : ' + str(payload_length))
            payload_raw = self.read(payload_length+4)
            payload = ['0x' + '{0:0>2X}'.format(x) for x in payload_raw]
            #payload = [hex(ord(x)) for x in payload_raw]
            if payload[0] == '0x12': 
                # then it's a mistake, we should wait before trying again
                if isDebug:
                    print('Device not ready, waiting a bit')
                time.sleep(0.1)
            # check checksum
            checksum = 0
            for x in payload[:-4]:
                checksum = checksum + int(x,16)
            checksum = checksum % (2**15)
            if int(''.join(payload[-4:-2]).replace('0x',''),16) != checksum:
                print("Error in received pattern")
                check = 1
            else:
                check = 0
        payload.pop()
        payload.pop()
        payload.pop()
        payload.pop()
        if isDebug:
            print("Received: " + str(payload))
        return(payload)

    def get_configuration(self):
        try:
            self.send(['0xB7'])
            self.conf = self.receive()
            return self.conf
        except:
            print("Can't get device configuration")
            return 0

    def get_id(self):
        try:
            self.send(['0xBF'])
            self.id = self.receive()
            return self.id
        except:
            print("Can't get device ID")
            return 0

class main:

    def __init__(self):
        '''User interface of the DG200 datalogger control software'''
        self.builder = Gtk.Builder()
        self.builder.add_from_file("Gui.ui") 
        self.window = self.builder.get_object("window_main")
        if self.window:
            self.window.connect("destroy", self.quit)
        self.entry_detect = self.builder.get_object("entry_detect")
        self.radiobutton_ptds = self.builder.get_object("radiobutton_ptds")
        self.radiobutton_ptdsa = self.builder.get_object("radiobutton_ptdsa")
        self.radiobutton_by_time = self.builder.get_object("radiobutton_by_time")
        self.radiobutton_by_distance = \
                self.builder.get_object("radiobutton_by_distance")
        self.checkbutton_disable_speed = \
                self.builder.get_object("checkbutton_disable_speed")
        self.checkbutton_disable_distance = \
                self.builder.get_object("checkbutton_disable_distance")
        self.checkbutton_waas = self.builder.get_object("checkbutton_waas")
        self.entry_speed_threshold = \
                self.builder.get_object('entry_speed_threshold')
        self.entry_distance_threshold = \
                self.builder.get_object('entry_distance_threshold')
        self.entry_time_interval = \
                self.builder.get_object('entry_time_interval')
        self.entry_distance_interval = \
                self.builder.get_object('entry_distance_interval')
        self.label_memory_usage = \
                self.builder.get_object('label_memory_usage')
        self.progress_bar = \
                self.builder.get_object('progressbar')
        self.button_get_track_list= \
                self.builder.get_object("button_get_track_list")
        self.button_select_all= \
                self.builder.get_object("button_select_all")
        self.button_select_none= \
                self.builder.get_object("button_select_none")
        self.button_download= \
                self.builder.get_object("button_download")
        self.button_clear_mem= \
                self.builder.get_object("button_clear_mem")
        self.button_get_conf= \
                self.builder.get_object("button_get_conf")
        self.button_apply_conf= \
                self.builder.get_object("button_apply_conf")
        ## Generate treeview
        # Create treestore
        self.treestore = Gtk.TreeStore(bool,str,str,int,int)
        # Get treeview and set model
        self.treeview = self.builder.get_object('treeview')
        self.treeview.set_model(self.treestore)
        # Create columns
        self.check_column = Gtk.TreeViewColumn('select')
        self.treeview.append_column(self.check_column)
        self.check_cell = Gtk.CellRendererToggle()
        self.check_column.pack_start(self.check_cell, False)
        self.check_column.add_attribute(self.check_cell,'active',0)
        self.check_cell.connect("toggled", self.toggled_cb, (self.treestore, 0))
        self.date_column = Gtk.TreeViewColumn('Date')
        self.treeview.append_column(self.date_column)
        self.date_cell = Gtk.CellRendererText()
        self.date_column.pack_start(self.date_cell,True)
        self.date_column.add_attribute(self.date_cell,'text',1)
        self.date_column.set_expand(True)
        self.time_column = Gtk.TreeViewColumn('Time')
        self.treeview.append_column(self.time_column)
        self.time_cell = Gtk.CellRendererText()
        self.time_column.pack_start(self.time_cell,True)
        self.time_column.add_attribute(self.time_cell,'text',2)
        self.time_column.set_expand(True)
        self.parts_column = Gtk.TreeViewColumn('Parts')
        self.treeview.append_column(self.parts_column)
        self.parts_cell = Gtk.CellRendererText()
        self.parts_column.pack_start(self.parts_cell,True)
        self.parts_column.add_attribute(self.parts_cell,'text',4)

        dict = {"on_button_quit_clicked": self.quit,
                "on_button_detect_clicked": self.detect,
                "on_button_connect_clicked": self.connect,
                "on_button_get_conf_clicked": self.get_configuration,
                "on_button_clear_mem_clicked": self.clear_memory,
                "on_button_apply_conf_clicked": self.set_configuration,
                "on_button_get_track_list_clicked": self.get_track_list,
                "on_button_download_clicked": self.download_tracks,
                "on_button_select_all_clicked": self.select_all,
                "on_button_select_none_clicked": self.select_none,
               }
        self.builder.connect_signals(dict)

    def toggled_cb(self,cell, path, user_data):
        model, column = user_data
        if path.find(':') == -1: # Then it's a mother line
            for i in range(0,model.iter_n_children(model.get_iter_from_string(path))):
                model[path + ':' + str(i)][column] = not model[path][column]
        model[path][column] = not model[path][column]
        return

    def select_all(self, widget):
        for row in self.treestore:
            row[0] = True

    def select_none(self, widget):
        for row in self.treestore:
            row[0] = False
         
    def set_sensitive(self):
        self.button_get_track_list.set_sensitive(True)
        self.button_select_all.set_sensitive(True)
        self.button_select_none.set_sensitive(True)
        self.button_download.set_sensitive(True)
        self.button_clear_mem.set_sensitive(True)
        self.radiobutton_ptds.set_sensitive(True)
        self.radiobutton_ptdsa.set_sensitive(True)
        self.radiobutton_by_time.set_sensitive(True)
        self.radiobutton_by_distance.set_sensitive(True)
        self.checkbutton_waas.set_sensitive(True)
        self.checkbutton_disable_speed.set_sensitive(True)
        self.checkbutton_disable_distance.set_sensitive(True)
        self.entry_time_interval.set_sensitive(True)
        self.entry_distance_interval.set_sensitive(True)
        self.entry_speed_threshold.set_sensitive(True)
        self.entry_distance_threshold.set_sensitive(True)
        self.button_get_conf.set_sensitive(True)
        self.button_apply_conf.set_sensitive(True)

    def detect(self,widget):
        '''Tries to detect the presence of DG200'''
        if isDebug:
            print("Detects the DG200")
        self.list_ttyUSB = []
        for x in range(0,10):
            if os.path.exists('/dev/ttyUSB' + str(x)):
                try:
                    test = DG200()
                    test.connect('/dev/ttyUSB' + str(x)) 
                    if test.get_id() != 0:
                        self.list_ttyUSB.append('/dev/ttyUSB' + str(x))
                    test.close()
                except:
                    test.close()
                    pass
        self.entry_detect.set_text(self.list_ttyUSB[0])
        if isDebug:
            print("DG200 detected on " + str(self.list_ttyUSB))

    def connect(self,widget):
        '''Connection to the specified address'''
        if isDebug:
            print("Try connection")
        self.dg200 = DG200()
        res = self.dg200.connect(self.entry_detect.get_text())
        if res == 1:
            self.set_sensitive()
            self.get_configuration(None)

    def get_configuration(self,widget):
        '''Get the configuration of th device'''
        if isDebug:
            print('Get configuration')
        #try:
        self.dg200.get_configuration()
        if self.dg200.conf != 0:
            # Information type
            if bytes2int(self.dg200.conf[1]) == 1:
                self.radiobutton_ptds.set_active(1)
            elif bytes2int(self.dg200.conf[1]) == 2:
                self.radiobutton_ptdsa.set_active(1)
            # Interval by time or distance
            if bytes2int(self.dg200.conf[26]) == 0:
                self.radiobutton_by_time.set_active(True)
            else:
                self.radiobutton_by_distance.set_active(True)
            # Time interval
            self.entry_time_interval.set_text(
                    str(bytes2int(self.dg200.conf[12:16])/1000))
            # Distance interval
            self.entry_distance_interval.set_text(
                    str(bytes2int(self.dg200.conf[29:33])))
            # Speed threshold flag
            if bytes2int(self.dg200.conf[2]) == 1:
                self.checkbutton_disable_speed.set_active(True)
            # Speed threshold
            self.entry_speed_threshold.set_text(
                    str(bytes2int(self.dg200.conf[3:7])))
            # Distance threshold flag
            if bytes2int(self.dg200.conf[7]) == 1:
                self.checkbutton_disable_distance.set_active(True)
            # Distance threshold
            self.entry_distance_threshold.set_text(
                    str(bytes2int(self.dg200.conf[8:12])))
            # WAAS flag
            if bytes2int(self.dg200.conf[42]) == 1:
                self.checkbutton_waas.set_active(True)
            self.label_memory_usage.set_text(
                    'Memory usage: ' + str(bytes2int(self.dg200.conf[43])) + '%')

    def set_configuration(self,widget):
        '''Apply new configuration from the GUI to the device'''
        print('Set configuration')
        payload = ['0xB8']
        # Information type
        if self.radiobutton_ptdsa.get_active():
            payload.append('0x02')
        else:
            payload.append('0x01')
        # Speed threshold flag
        if self.checkbutton_disable_speed.get_active():
            payload.append('0x01')
        else:
            payload.append('0x00')
        # Speed threshold
        try:
            speed_threshold = \
                    int2bytes(int(self.entry_speed_threshold.get_text()),4)
        except:
            speed_threshold = ['0x00', '0x00', '0x00', '0x00']
        payload.extend(speed_threshold)
        # Distance threshold flag
        if self.checkbutton_disable_distance.get_active():
            payload.append('0x01')
        else:
            payload.append('0x00')
        #Distance threshold
        try:
            distance_threshold = \
                    int2bytes(int(self.entry_distance_threshold.get_text()),4)
        except:
            distance_threshold = ['0x00', '0x00', '0x00', '0x00']
        payload.extend(distance_threshold)
        # Time interval
        try:
            time_interval = \
                    int2bytes(1000 * int(self.entry_time_interval.get_text()),4)
        except:
            time_interval = ['0x00', '0x00', '0x00', '0x00']
        payload.extend(time_interval)
        # Lot of unused bytes
        payload.extend(
                ['0x00','0x00','0x00','0x00','0x00','0x00','0x00','0x00','0x00','0x00'])
        # Interval by time/distance flag
        if self.radiobutton_by_time.get_active():
            payload.append('0x00')
        else:
            payload.append('0x01')
        # Lot of unused bytes
        payload.extend(['0x00','0x00'])
        # Interval by distance
        try:
            distance_interval = \
                    int2bytes(int(self.entry_distance_interval.get_text()),4)
        except:
            distance_interval = ['0x00', '0x00', '0x00', '0x00']
        payload.extend(distance_interval)
        # Lot of unused bytes
        payload.extend(['0x00','0x00','0x00','0x00','0x00','0x00','0x00','0x00'])
        # Operation mode should be 4
        payload.append('0x04')
        # WAAS
        if self.checkbutton_waas.get_active():
            payload.append('0x01')
        else:
            payload.append('0x00')
        # Send command and check if everything is correct
        self.dg200.send(payload)
        test = self.dg200.receive()

    def get_track_list(self,widget):
        '''get track list from the header files'''
        if isDebug:
            print("Get track list")
        self.treestore.clear()
        self.dg200.send(['0xBB', '0x00', '0x00']) # get first header command
        header_list_tmp = self.dg200.receive()
        header_list = header_list_tmp[5:] # Remove the first bytes (number of headers)
        self.header_index = []
        header_iter = None
        header_track_num = 0 # number of track components in one track
        next_tracker_index = bytes2int(header_list_tmp[3:5]) # Index of next track header
        while next_tracker_index != 0: 
            # if it's zero, then there is no more header iteration
            # Get next header file
            self.dg200.send(['0xBB', header_list_tmp[3], header_list_tmp[4]])
            header_list_tmp = self.dg200.receive()
            next_tracker_index = bytes2int(header_list_tmp[3:5]) 
            header_list.extend(header_list_tmp[5:]) # Whole track list is built

        if isDebug:
            print("header_list: " + str(header_list))
        # Now we process this header_list
        nb_header = round(len(header_list)/12)
        for num_header in range(0,nb_header):
            header_date_raw = '{0:0>6d}'.format(
                    bytes2int(header_list[4+12*num_header:8+12*num_header]))
            header_date = '/'.join([
                    header_date_raw[0:2],
                    header_date_raw[2:4],
                    header_date_raw[4:6]])
            # time is bytes 0-4, but first byte is used to detect first track
            header_time_raw = '{0:0>6d}'.format(
                    bytes2int(header_list[1+12*num_header:4+12*num_header]))
            header_time = ':'.join([
                    header_time_raw[0:2],
                    header_time_raw[2:4],
                    header_time_raw[4:6]])
            # We want to keep the header_index, thus the self
            self.header_index.append(bytes2int(header_list[8 + 12*num_header : 12 +
                    12*num_header]))
            if header_list[12*num_header] == '0x80':
                # header is first in its session
                if header_iter: 
                    # set number of track components of the previous header
                    self.treestore.set_value(header_iter, 4, header_track_num)
                    header_track_num = 0

                header_iter = self.treestore.append(
                        None,[True,header_date,header_time,self.header_index[num_header],0])
            # increase track components number
            header_track_num += 1

            if num_header == nb_header - 1: # close things at the end
                self.treestore.set_value(header_iter, 4, header_track_num)

    def download_tracks(self,widget):
        '''Find which tracks to download and launch the downloader'''
        if isDebug:
            print("download files")
        # open dialog for setting the download folder
        open_dialog = Gtk.FileChooserDialog(title=None, parent=self.window,
                action=Gtk.FileChooserAction.SELECT_FOLDER,
                buttons=(Gtk.STOCK_CANCEL,Gtk.ResponseType.CANCEL,Gtk.STOCK_OPEN,Gtk.ResponseType.OK))
        open_dialog.set_current_folder(os.path.expanduser("~"))
        open_dialog.set_show_hidden(False)
        open_dialog.set_title("Choose download directory")
        res = open_dialog.run()
        if res == Gtk.ResponseType.OK: # OK button clicked
            self.nbtrackparts = 0
            self.progress_counter = 0
            self.folder = open_dialog.get_filename()
            open_dialog.destroy()
            # calculate the number of track parts to download
            for row in self.treestore: # for each track
                if row[0]: # if selected
                    self.nbtrackparts = self.nbtrackparts + row[4] 
            # download tracks
            for row in self.treestore: # for each track
                if row[0]: # if selected
                    # Gets the indices of its components
                    first_index = self.header_index.index(row[3])
                    if isDebug:
                        print('date: ' + row[1] + ' ' + row[2] +  \
                                ", track components: " + \
                                str(self.header_index[first_index:first_index + row[4]]))
                    # Go get the track
                    self.get_track(self.header_index[first_index:first_index + row[4]])
            # Finished, set progressbar to 0
            self.progress_bar.set_fraction(float(0))
            # and get configuration for the diode to switch on
            self.dg200.get_configuration()
            while Gtk.events_pending():
                Gtk.main_iteration()
        else:
            open_dialog.destroy()

    def get_track(self,list_index):
        '''Download one track'''
        track = []
        for index in list_index:
            payload = ['0xB5'] # get track command
            payload.extend(int2bytes(index,2)) # specify index of track component
            self.dg200.send(payload)
            first_part = self.dg200.receive()
            first_part.pop(0) # remove command byte
            self.progress_counter += 1
            self.progress_bar.set_fraction(float(self.progress_counter)/float(2*self.nbtrackparts))
            while Gtk.events_pending():
                Gtk.main_iteration()
            second_part = self.dg200.receive()
            second_part.pop(0)
            self.progress_counter += 1
            self.progress_bar.set_fraction(float(self.progress_counter)/float(2*self.nbtrackparts))
            while Gtk.events_pending():
                Gtk.main_iteration()
            track_raw = first_part + second_part

            # processing of the raw data (remove '0xFF' and convert to list)
            if bytes2int(track_raw[28:32]) == 1:
                # Then it's a 20bytes coding (pos,date/time,speed)
                del(track_raw[-16:]) # remove the 16 trailing FFs
                while track_raw[-20:] == ['0xFF' for x in range(0,20)]:
                    del(track_raw[-20:])

                # first point (which is in 32 bytes format)
                track.append(process_point(track_raw[:32]))
                nb_points = int((len(track_raw) - 32) / 20 + 1)
                # next points
                for point in range(1,nb_points):
                    track.append(process_point(track_raw[32+20*(point-1):32+20*point]))

            elif bytes2int(track_raw[28:32]) == 2:
                ## Then it's a 32bytes coding (pos,date/time,speed,altitude)
                while track_raw[-32:] == ['0xFF' for x in range(0,32)]:
                    del(track_raw[-32:])

                nb_points = int(len(track_raw)/32)
                for point in range(0,nb_points):
                    track.append(process_point(track_raw[32*point:32+32*point]))
        
        if isDebug:
            print("refined track = " + str(track))
        write_gpx(self.folder, track)

    def clear_memory(self,widget):
        '''Clears the memory, after asking for confirmation'''
        dialog = Gtk.MessageDialog(self.window,
                        Gtk.DialogFlags.DESTROY_WITH_PARENT,
                        type=Gtk.MessageType.WARNING,
                        buttons=Gtk.ButtonsType.CANCEL, 
                        message_format="Are you sure you want to erase all tracks on your DG200 data logger?")
        dialog.set_title('Clear memory?')
        dialog.add_button(Gtk.STOCK_OK, Gtk.ResponseType.OK)
        response = dialog.run()
        dialog.destroy()
        if response == Gtk.ResponseType.OK:
            self.dg200.send(['0xBA', '0xFF', '0xFF'])
            res = self.dg200.receive()
            if bytes2int(res[1:5]) == 0: #bug 
                print("Memory cleared")
        self.dg200.get_configuration()
        self.get_track_list(None)

    def quit(self,widget):
        try:
            self.dg200.close()
        except: 
            pass
        Gtk.main_quit()


        
# Execution du programme
if __name__ == "__main__":
    main()
    Gtk.main()

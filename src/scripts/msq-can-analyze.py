#!/usr/bin/env python3
import argparse
import sys
import can
from termcolor import colored, cprint
from collections import defaultdict

MSG_CMD = 0
MSG_REQ = 1
MSG_RSP = 2
MSG_XSUB = 3
MSG_BURN = 4
OUTMSG_REQ = 5
OUTMSG_RSP = 6
MSG_XTND = 7

MSG_XTND_FWD = 8
MSG_XTND_CRC = 9
MSG_XTND_REQX = 12
MSG_XTND_BURNACK = 14
MSG_XTND_PROT = 0x80
MSG_XTND_WCR = 0x81
MSG_XTND_SPND = 0x82

MSG_TYPE_NAME_LUT = {
	MSG_CMD : "MSG_CMD",
	MSG_REQ : "MSG_REQ",
	MSG_RSP : "MSG_RSP",
	MSG_XSUB : "MSG_XSUB",
	MSG_BURN : "MSG_BURN",
	OUTMSG_REQ : "OUTMSG_REQ",
	OUTMSG_RSP : "OUTMSG_RSP",
	MSG_XTND : "MSG_XTND"
}
XTYPE_NAME_LUT = {
	MSG_XTND_FWD : "FWD",
	MSG_XTND_CRC : "CRC",
	MSG_XTND_REQX : "REQX",
	MSG_XTND_BURNACK : "BURNACK",
	MSG_XTND_PROT : "PROT",
	MSG_XTND_WCR : "WCR",
	MSG_XTND_SPND : "SPND"
}
def _type_to_name(type):
	try:
		return MSG_TYPE_NAME_LUT[type]
	except KeyError:
		return "***UNKNOWN MSG TYPE (%s)***" % type

def _xtnd_type_to_name(xtype):
	try:
		return XTYPE_NAME_LUT[xtype]
	except KeyError:
		return "***UNKNOWN XTND TYPE (%s)***" % xtype

class MSQ_ExtendedId(object):
	def __init__(self, can_id):
		self.can_id = can_id

		self.table = ((can_id >> 3) & 0xf) | ((can_id & 0x4) << 2)
		self.to_id = can_id >> 7 & 0xf
		self.from_id = can_id >> 11 & 0xf
		self.type = can_id >> 15 & 0x7
		self.offset = can_id >> 18 & 0x7ff

	def type_name(self):
		return _type_to_name(self.type)

	def __repr__(self):
		return "<{}>".format(self.type_name())

class MSQ_MsgBase(object):
	def __init__(self, can_msg, msq_id=None):
		self.can_msg = can_msg
		if msq_id is MSQ_ExtendedId:
			self.msq_id = msq_id
		else:
			self.msq_id = MSQ_ExtendedId(can_msg.arbitration_id)

	def data(self):
		return self.can_msg.data

	def time(self):
		return self.can_msg.timestamp

	def log_event(self,**kwargs):
		no_time = kwargs.get('no_time',False)
		timestamp = ""
		if not no_time:
			timestamp = "%.6f | " % (self.time())
		ctext = colored("%s%s: %d --> %d;" %
			(timestamp,self.msq_id.type_name(),self.msq_id.from_id,self.msq_id.to_id), 'white')
		print(ctext)

	def __repr__(self):
		return "<{}>".format(self.msq_id.type_name())

class MSQ_MsgCMD(MSQ_MsgBase):
	def __init__(self, can_msg, msq_id=None):
		super(MSQ_MsgCMD, self).__init__(can_msg,msq_id)

	def cmd_size(self):
		return len(self.data())

	def log_event(self,**kwargs):
		ascii_data = kwargs.get('ascii_data',False)
		no_time = kwargs.get('no_time',False)
		timestamp = ""
		if not no_time:
			timestamp = "%.6f | " % (self.time())
		data_str = self.data().hex()
		if ascii_data:
			data_str += ' | %s' %  self.data().decode('utf-8',errors='ignore').encode('ascii',errors='ignore')
		ctext = colored("%s%s: %d --> %d; table = %2d; offset = %2d; cmd_size = %d; data = 0x%s" %
			(timestamp,self.msq_id.type_name(),self.msq_id.from_id,self.msq_id.to_id,
			 self.msq_id.table,self.msq_id.offset,self.cmd_size(),data_str), 'magenta')
		print(ctext)

class MSQ_MsgREQ(MSQ_MsgBase):
	def __init__(self, can_msg, msq_id=None):
		super(MSQ_MsgREQ, self).__init__(can_msg,msq_id)

	def req_size(self):
		return self.data()[2] & 0xf

	def log_event(self,**kwargs):
		no_time = kwargs.get('no_time',False)
		timestamp = ""
		if not no_time:
			timestamp = "%.6f | " % (self.time())
		ctext = colored("%s%s: %d --> %d; table = %2d; offset = %2d; req_size = %d" %
			(timestamp,self.msq_id.type_name(),self.msq_id.from_id,self.msq_id.to_id,
			 self.msq_id.table,self.msq_id.offset,self.req_size()), 'blue')
		print(ctext)

class MSQ_MsgRSP(MSQ_MsgBase):
	def __init__(self, can_msg, msq_id=None):
		super(MSQ_MsgRSP, self).__init__(can_msg,msq_id)

	def rsp_size(self):
		return len(self.data())

	def log_event(self,**kwargs):
		ascii_data = kwargs.get('ascii_data',False)
		no_time = kwargs.get('no_time',False)
		timestamp = ""
		if not no_time:
			timestamp = "%.6f | " % (self.time())
		data_str = self.data().hex()
		if ascii_data:
			data_str += ' | %s' %  self.data().decode('utf-8',errors='ignore').encode('ascii',errors='ignore')
		ctext = colored("%s%s: %d <-- %d; table = %2d; offset = %2d; rsp_size = %d; data = 0x%s" %
			(timestamp,self.msq_id.type_name(),self.msq_id.to_id,self.msq_id.from_id,
			 self.msq_id.table,self.msq_id.offset,self.rsp_size(),data_str), 'green')
		print(ctext)

class MSQ_MsgXTND(MSQ_MsgBase):
	def __init__(self, can_msg, msq_id=None):
		super(MSQ_MsgXTND, self).__init__(can_msg,msq_id)

	def xtnd_type(self):
		return self.data()[0]

	def xtnd_type_name(self):
		return _xtnd_type_to_name(self.xtnd_type())

	def full_type_name(self):
		return "%s(%s)" % (self.msq_id.type_name(),self.xtnd_type_name())

	def log_event(self,**kwargs):
		no_time = kwargs.get('no_time',False)
		timestamp = ""
		if not no_time:
			timestamp = "%.6f | " % (self.time())
		ctext = colored("%s%s: %d --> %d;" %
			(timestamp,self.full_type_name(),self.msq_id.from_id,self.msq_id.to_id), 'yellow')
		print(ctext)

def _get_msq_msg(can_msg):
	msq_id = MSQ_ExtendedId(can_msg.arbitration_id)
	if msq_id.type == MSG_CMD:
		return MSQ_MsgCMD(can_msg,msq_id)
	elif msq_id.type == MSG_REQ:
		return MSQ_MsgREQ(can_msg,msq_id)
	elif msq_id.type == MSG_RSP:
		return MSQ_MsgRSP(can_msg,msq_id)
	elif msq_id.type == MSG_XTND:
		return MSQ_MsgXTND(can_msg,msq_id)
	return MSQ_MsgBase(can_msg,msq_id)

def main():
	parser = argparse.ArgumentParser(
		prog='msg-can-analyze',
		usage='%(prog)s [options]',
		description="parses Megasquirt CAN bus traffic")

	parser.add_argument(
		'--device',
		type=str,default="can0")

	parser.add_argument(
		'--ignore-std',
		help="if set, analyzer will ignore 11bit (standard) CAN frames",
		action='store_true',default=False)

	parser.add_argument(
		'--no-time',
		help="if set, data dumps will not contain timestamps",
		action='store_true',default=False)

	parser.add_argument(
		'--ascii-data',
		help="if set, data dumps will be logged in ASCII encoding too",
		action='store_true',default=False)

	args = parser.parse_args()

	pending_req_by_id = defaultdict(lambda: None)

	can_bus = can.interface.Bus(args.device, bustype='socketcan')
	while True:
		msg = can_bus.recv()
		if msg.is_extended_id:
			ms_msg = _get_msq_msg(msg)
			ms_msg.log_event(ascii_data=args.ascii_data, no_time=args.no_time)

			# perform REQ/RSP error detection
			from_id = ms_msg.msq_id.from_id
			to_id = ms_msg.msq_id.to_id
			if isinstance(ms_msg,MSQ_MsgREQ):
				if pending_req_by_id[to_id] != None:
					print(colored('Back-to-back REQ! message dropped?','red'))
				pending_req_by_id[to_id] = ms_msg
			elif isinstance(ms_msg,MSQ_MsgRSP):
				pend_req = pending_req_by_id[from_id]
				if pend_req == None:
					print(colored('Unsolicited RSP!','red'))
				else:
					# sanity check the RSP w/re to original REQ
					if pend_req.req_size() != ms_msg.rsp_size():
						print(colored('mismatch response size!','red'))
				# mark req no longer pending
				pending_req_by_id[from_id] = None
		elif args.ignore_std:
			pass
		else:
			print(msg)

if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		pass

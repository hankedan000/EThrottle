#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
from collections import defaultdict
from datetime import datetime
from enum import Enum

COL_TYPE = 0
COL_NAME = 1
COL_BIT_WIDTH = 2
COL_CONST_VALUE = 3
COL_ARRAY_SIZE = 4
COL_CODE_COMMENT = 5
COL_PAGE_ID = 6
COL_UNITS = 7
COL_SCALE = 8
COL_TRANSLATE = 9
COL_LO = 10
COL_HI = 11
COL_DECIMAL_DIGITS = 12
COL_GAUGE_CATEGORY = 13
COL_GAUGE_TITLE = 14
COL_LO_DANGER = 15
COL_LO_WARN = 16
COL_HI_WARN = 17
COL_HI_DANGER = 18
COL_VD = 19
COL_LD = 20

DEFAULT_INDENT = "    "

TRIVIAL_TYPE_LUT = {
	'uint8_t' : {'size_bytes' : 1, 'ini_type' : 'U08'},
	'int8_t' : {'size_bytes' : 1, 'ini_type' : 'S08'},
	'uint16_t' : {'size_bytes' : 2, 'ini_type' : 'U16'},
	'int16_t' : {'size_bytes' : 2, 'ini_type' : 'S16'},
	'uint32_t' : {'size_bytes' : 4, 'ini_type' : 'U32'},
	'int32_t' : {'size_bytes' : 4, 'ini_type' : 'S21'}
}

def is_constant_type(type_name):
	return type_name.startswith("const")

def is_trivial_type(type_name):
	return type_name in TRIVIAL_TYPE_LUT

def is_membered_type(type_name):
	MEMBERED_TYPES_LUT = ['struct', 'union', 'bitfield']
	return type_name in MEMBERED_TYPES_LUT

def parse_number(nspace, numb_type, value_str, **kwargs):
	default = kwargs.get('default', None)
	allow_constants = kwargs.get('allow_constants', False)

	if len(value_str) == 0:
		return default
	try:
		return numb_type(value_str)
	except ValueError as ve:
		if allow_constants:
			# try to lookup constant value by name
			return numb_type(nspace.get_constant(value_str).const_value)
		else:
			raise ve

def bits2bytes(n_bits, round_func):
	return round_func(n_bits / 8.0)

def make_padded_row(row, col_widths, **kwargs):
	is_comment = kwargs.get('is_comment', False)
	default_delim = kwargs.get('default_delim', ', ')
	delim_by_col = kwargs.get('delim_by_col', {})

	row_str = ""
	for cc,col in enumerate(row):
		width = col_widths[cc]
		if cc == 0 and width > 0 and is_comment:
			# steal 1 to prevent comment char ';' from misaligning
			width -= 1
		if col == None:
			row_str += "".ljust(width)
		else:
			row_str += col.ljust(width)
		skip_delim = (cc+1) < len(row) and row[cc+1] == None
		if skip_delim:
			row_str += "  "
		elif cc in delim_by_col:
			row_str += delim_by_col[cc]
		elif cc < (len(row) - 1):
			row_str += default_delim
	return row_str

def make_padded_table(rows, **kwargs):
	col_titles = kwargs.get('col_titles', [])
	indent = kwargs.get('indent', "")
	default_delim = kwargs.get('default_delim', ', ')
	delim_by_col = kwargs.get('delim_by_col', {})
	row_prefix = kwargs.get('row_prefix', '')
	row_suffix = kwargs.get('row_suffix', '')

	col_widths = []
	def update_col_widths(row):
		for cc,col in enumerate(row):
			width = 0
			if col != None:
				width = len(col)
			if cc < len(col_widths):
				col_widths[cc] = max(col_widths[cc], width)
			else:
				col_widths.append(width)
	
	if len(col_titles) > 0:
		update_col_widths(col_titles)
	for row in rows:
		update_col_widths(row)

	table_str = ""
	if len(col_titles) > 0:
		table_str += indent + "; *************************************************************************************************************************************\n"
		table_str += indent + ";" + make_padded_row(col_titles, col_widths, delim_by_col=delim_by_col, default_delim=default_delim, is_comment=True) + '\n'

	for idx,row in enumerate(rows):
		if idx > 0:
			table_str += "\n"
		table_str += indent + row_prefix
		table_str += make_padded_row(row, col_widths, delim_by_col=delim_by_col, default_delim=default_delim)
		table_str += row_suffix

	return table_str

# object containing data parsed from a single row of the source CSV
class RowObject(object):
	def __init__(self, nspace, row):
		self.csv_row = row
		self.type_name = row[COL_TYPE]
		self.name = row[COL_NAME]
		self.bit_width = parse_number(nspace, int, row[COL_BIT_WIDTH])
		self.const_value = row[COL_CONST_VALUE]
		self.array_size = parse_number(nspace, int, row[COL_ARRAY_SIZE])
		self.code_comment = row[COL_CODE_COMMENT]
		self.page_id = row[COL_PAGE_ID]
		self.units = row[COL_UNITS]
		self.scale = parse_number(nspace, float, row[COL_SCALE], default=1.0)
		self.translate = parse_number(nspace, float, row[COL_TRANSLATE], default=0.0)
		self.lo = parse_number(nspace, float, row[COL_LO], allow_constants=True, default=0.0)
		self.hi = parse_number(nspace, float, row[COL_HI], allow_constants=True, default=1.0)
		self.decimal_digits = parse_number(nspace, int, row[COL_DECIMAL_DIGITS], default=0)
		self.gauge_category = row[COL_GAUGE_CATEGORY]
		self.gauge_title = row[COL_GAUGE_TITLE]
		self.lo_danger = parse_number(nspace, float, row[COL_LO_DANGER], allow_constants=True)
		self.lo_warn = parse_number(nspace, float, row[COL_LO_WARN], allow_constants=True)
		self.hi_warn = parse_number(nspace, float, row[COL_HI_WARN], allow_constants=True)
		self.hi_danger = parse_number(nspace, float, row[COL_HI_DANGER], allow_constants=True)
		self.vd = parse_number(nspace, int, row[COL_VD], default=0)
		self.ld = parse_number(nspace, int, row[COL_LD], default=0)

		self.ini_class = 'scalar'
		if self.is_array():
			self.ini_class = 'array'
		elif self.is_bitfield():
			self.ini_class = 'bits'

		self.type_ref = None
		if self.is_constant():
			pass
		elif self.is_trivial_type():
			pass
		elif self.is_membered_type():
			pass
		else:
			if nspace.has_type_name(self.type_name):
				self.type_ref = nspace.get_type_by_name(self.type_name)
			else:
				raise RuntimeError("Unable to determine RowObject's type reference. type = '%s', name = %s" % (self.type_name, self.name))

	def is_constant(self):
		return is_constant_type(self.type_name)
	
	def is_trivial_type(self):
		return is_trivial_type(self.type_name)
	
	def is_array(self):
		return self.array_size != None and self.array_size > 0
	
	def is_bitfield(self):
		return self.bit_width != None and self.bit_width > 0
	
	def trivial_c_type_name(self):
		if self.is_constant():
			return str(self.type_name).replace("const ", "")
		elif self.is_trivial_type():
			return str(self.type_name)
		else:
			raise RuntimeError("can't get trivial type name for membered types")
		
	def trivial_ini_type_name(self):
		return TRIVIAL_TYPE_LUT[self.trivial_c_type_name()]['ini_type']
		
	def size_bytes(self):
		return self.size_bits() / 8.0
	
	def size_bits(self):
		size_bits = 0
		if self.is_bitfield():
			size_bits = self.bit_width
		else:
			size_bits = TRIVIAL_TYPE_LUT[self.trivial_c_type_name()]['size_bytes'] * 8
		if self.is_array():
			size_bits *= self.array_size
		return size_bits
	
	def is_membered_type(self):
		return is_membered_type(self.type_name)
	
	def get_c_type_string(self, **kwargs):
		indent = kwargs.get('indent', DEFAULT_INDENT)
		code_comment = kwargs.get('code_comment', True)
		scale_trans_comment = kwargs.get('scale_trans_comment', True)
		range_comment = kwargs.get('range_comment', True)
		show_units = kwargs.get('show_units', True)
		text = ""

		if code_comment and len(self.code_comment) > 0:
			text += "%s// %s\n" % (indent, self.code_comment)
		if scale_trans_comment and (self.scale != 1.0 or self.translate != 0.0):
			text += "%s// scale: %f; translate: %f\n" % (indent, self.scale, self.translate)
		if range_comment and self.lo != None and self.hi != None:
			text += "%s// range: [%f,%f]\n" % (indent, self.lo, self.hi)
		if show_units and len(self.units) > 0:
			text += "%s// units: %s\n" % (indent, self.units)

		if self.bit_width:
			text += "%s%s %s : %d;" % (indent, self.type_name, self.name, self.bit_width)
		else:
			text += "%s%s %s;" % (indent, self.type_name, self.name)
		return text

class MemberedType(object):
	def __init__(self, row_obj):
		self.row_obj = row_obj
		self.name = row_obj.name
		self.type_name = row_obj.type_name
		self.members_by_name = dict()
		self.members = []

	def add_member(self, row_obj):
		if row_obj.name in self.members_by_name:
			raise RuntimeError("WARN: member with name '%s' already exists in Struct '%s'." % (row_obj.name, self.name))
		self.members_by_name[row_obj.name] = row_obj
		self.members.append(row_obj)

	def get_c_type_string(self, **kwargs):
		indent = kwargs.get('indent',"")

		str = ""
		if self.row_obj.type_name == "bitfield":
			str = "struct %s {\n" % (self.row_obj.name)
		elif self.row_obj.type_name == "struct":
			str = "struct %s {\n" % (self.row_obj.name)
		elif self.row_obj.type_name == "union":
			str = "union %s {\n" % (self.row_obj.name)
		
		for idx, member in enumerate(self.members):
			if idx > 0:
				str += "\n"
			str += member.get_c_type_string(indent=indent + DEFAULT_INDENT) + "\n"

		if len(str) > 0:
			str += "};"

		return str
	
	def get_ini_page_defn_rows(self, nspace, **kwargs):
		name_prefix = kwargs.get('name_prefix', "")
		offset = kwargs.get('offset', 0)

		rows = []

		my_size_bits = 0
		n_members = len(self.members)
		for idx, member in enumerate(self.members):
			member_size_bits = 0
			if member.is_trivial_type():
				row = []
				row.append(name_prefix + member.name)
				row.append(member.ini_class)
				row.append(member.trivial_ini_type_name())
				row.append('%d' % (offset + bits2bytes(my_size_bits, math.floor)))
				if member.ini_class == 'scalar' or member.ini_class == 'array':
					if member.ini_class == 'scalar':
						row.append(None)
					else:
						row.append('[%d]' % (member.array_size))
					row.append('"%s"' % member.units)
					row.append('%0.6f' % member.scale)
					row.append('%0.6f' % member.translate)
					row.append('%0.6f' % member.lo)
					row.append('%0.6f' % member.hi)
					row.append('%d' % member.decimal_digits)
				elif member.ini_class == 'bits':
					word_c_type = self.members[0].trivial_c_type_name()
					word_size_bits = int(TRIVIAL_TYPE_LUT[word_c_type]['size_bytes'] * 8)
					first_bit = int(my_size_bits) % word_size_bits
					last_bit = first_bit + member.bit_width - 1
					row.append('[%d:%d]' % (first_bit, last_bit))
					if len(member.units) > 0:
						units_str = ""
						for idx, bit_unit in enumerate(member.units.split(',')):
							if idx > 0:
								units_str += ","
							units_str += '"%s"' % bit_unit
						row.append(units_str)
				else:
					raise RuntimeError('unsupported ini_class "%s"' % member.ini_class)
				rows.append(row)
				member_size_bits = member.size_bits()
			else:
				this_prefix = name_prefix + member.name + "_"
				membered_type = nspace.get_type_by_name(member.type_name)
				(member_rows, member_size_bytes) = membered_type.get_ini_page_defn_rows(nspace, name_prefix=this_prefix, offset=offset+bits2bytes(my_size_bits, math.ceil))
				for member_row in member_rows:
					rows.append(member_row)
				member_size_bits = member_size_bytes * 8
			
			if self.row_obj.type_name == 'union':
				my_size_bits = max(my_size_bits, member_size_bits)
			else:
				my_size_bits += member_size_bits

		return (rows, bits2bytes(my_size_bits, math.ceil))
	
	def get_ini_out_pc_defn_rows(self, nspace, **kwargs):
		name_prefix = kwargs.get('name_prefix', "")
		offset = kwargs.get('offset', 0)

		rows = []

		my_size_bits = 0
		n_members = len(self.members)
		for idx, member in enumerate(self.members):
			member_size_bits = 0
			if member.is_trivial_type():
				row = []
				row.append(name_prefix + member.name)
				row.append(member.ini_class)
				row.append(member.trivial_ini_type_name())
				row.append('%d' % (offset + bits2bytes(my_size_bits, math.floor)))
				if member.ini_class == 'scalar':
					row.append(None)
					row.append('"%s"' % member.units)
					row.append('%0.6f' % member.scale)
					row.append('%0.6f' % member.translate)
				elif member.ini_class == 'bits':
					word_c_type = self.members[0].trivial_c_type_name()
					word_size_bits = int(TRIVIAL_TYPE_LUT[word_c_type]['size_bytes'] * 8)
					first_bit = int(my_size_bits) % word_size_bits
					last_bit = first_bit + member.bit_width - 1
					row.append('[%d:%d]' % (first_bit, last_bit))
					if len(member.units) > 0:
						units_str = ""
						for idx, bit_unit in enumerate(member.units.split(',')):
							if idx > 0:
								units_str += ","
							units_str += '"%s"' % bit_unit
						row.append(units_str)
				elif member.ini_class == 'array':
					print("WARN: OUT_PC cannot contain array types. '%s' will be omitted from ini output" % member.name)
					row = []
				else:
					raise RuntimeError('unsupported ini_class "%s"' % member.ini_class)
				if len(row) > 0:
					rows.append(row)
				member_size_bits = member.size_bits()
			else:
				this_prefix = name_prefix + member.name + "_"
				membered_type = nspace.get_type_by_name(member.type_name)
				(member_rows, member_size_bytes) = membered_type.get_ini_out_pc_defn_rows(nspace, name_prefix=this_prefix, offset=offset+bits2bytes(my_size_bits, math.ceil))
				for member_row in member_rows:
					rows.append(member_row)
				member_size_bits = member_size_bytes * 8
			
			if self.row_obj.type_name == 'union':
				my_size_bits = max(my_size_bits, member_size_bits)
			else:
				my_size_bits += member_size_bits

		return (rows, bits2bytes(my_size_bits, math.ceil))
	
	def get_ini_page_string(self, nspace, **kwargs):
		indent = kwargs.get('indent',"")
		COL_TITLES = ['name', 'class', 'type', 'offset', 'shape', 'units', 'scale', 'translate', 'lo', 'hi', 'decimal digits']

		(rows, size_bytes) = self.get_ini_page_defn_rows(nspace)
		str = indent + "page = %s\n" % self.row_obj.page_id
		str += make_padded_table(rows, col_titles=COL_TITLES, indent=indent, delim_by_col={0:" = "})
		return str
	
	def get_ini_out_pc_string(self, nspace, **kwargs):
		indent = kwargs.get('indent',"")
		COL_TITLES = ['name', 'class', 'type', 'offset', 'shape', 'units', 'scale', 'translate']

		(rows, size_bytes) = self.get_ini_out_pc_defn_rows(nspace)
		return make_padded_table(rows, col_titles=COL_TITLES, indent=indent, delim_by_col={0:" = "})

def traverse_members(type, nspace, callback, **kwargs):
	name_prefix = kwargs.get('name_prefix', "")
	offset = kwargs.get('offset', 0)

	my_size_bits = 0
	for idx, member in enumerate(type.members):
		full_type_name = name_prefix + member.name
		member_size_bits = 0
		if member.is_trivial_type():
			member_size_bits = member.size_bits()
			if callback != None:
				callback(member, full_type_name, offset)
		else:
			next_prefix = full_type_name + "_"
			membered_type = nspace.get_type_by_name(member.type_name)
			member_size_bytes = traverse_members(membered_type, nspace, callback, name_prefix=next_prefix, offset=offset+bits2bytes(my_size_bits, math.ceil))
			member_size_bits = member_size_bytes * 8
		
		if type.row_obj.type_name == 'union':
			my_size_bits = max(my_size_bits, member_size_bits)
		else:
			my_size_bits += member_size_bits
			
	return bits2bytes(my_size_bits, math.ceil)

class Namespace(object):
	def __init__(self):
		self.constants_by_name = dict() # Constant keyed by name
		self.types_by_name = dict()     # types keyed by name
		self.tables = dict()    # tables keyed by name

		self._curr_type = None # the current type being processed

	def add_constant(self, new_const):
		if new_const.name in self.constants_by_name:
			print("WARN: constant with name '%s' already exists in namespace. over writing previous." % (new_const.name))
		self.constants_by_name[new_const.name] = new_const

	def get_constant(self, name):
		return self.constants_by_name[name]

	def add_type(self, new_type):
		if new_type.name in self.types_by_name:
			print("WARN: new_type with name '%s' already exists in namespace. over writing previous." % (new_type.name))
		self.types_by_name[new_type.name] = new_type
	
	def has_type_name(self, type_name):
		return type_name in self.types_by_name

	def get_type_by_name(self, type_name):
		return self.types_by_name[type_name]
	
	# processes a CSV row
	def process_row(self, row):
		if len(row[COL_TYPE]) == 0:
			return # skip empty rows
		
		row_obj = RowObject(self, row)
		
		if row_obj.is_constant():
			self.add_constant(row_obj)
		elif row_obj.is_membered_type():
			if self._curr_type:
				self.add_type(self._curr_type)
			self._curr_type = MemberedType(row_obj)
		elif self._curr_type:
			self._curr_type.add_member(row_obj)
		elif self._curr_type == None:
			raise RuntimeError("can't process field into an unknown type")
		else:
			raise RuntimeError("unsupported type_name '%s'" % (row_obj.type_name))
		
	def finalize(self):
		if self._curr_type:
			self.add_type(self._curr_type)

def write_header_file(args, nspace, table_list):
	header_out_file = args.header_out_file
	if header_out_file == None:
		header_out_file = open(os.path.splitext(args.csvfile.name)[0] + ".h", 'w')
	
	c_table_rows = []
	global_vars = []
	table_size_defns = []
	table_flash_offset_defns = []
	table_id_enum_values = []
	flash_offset = 0
	for row in table_list:
		c_row = []
		table_id_enum_values.append("eTI_%s" % row[4])
		table_flash_offset_str = "-1"
		if len(row[0]) > 0:
			table_size = traverse_members(nspace.get_type_by_name(row[0]), nspace, None)
			table_size_defns.append(["%s_SIZE" % row[4], "%d" % table_size])
			if row[3] == 'eFlash':
				table_flash_offset = flash_offset
				flash_offset += table_size
				table_flash_offset_str = "%s_FLASH_OFFSET" % row[4]
				table_flash_offset_defns.append([table_flash_offset_str, "%d" % table_flash_offset])

		# generate table pointer entry
		if row[3] == 'eFlash':
			c_row.append("MegaCAN::tempPage")
			table_flash_offset = flash_offset
		elif row[2] == 'nullptr':
			c_row.append(row[2])
		elif len(row[2]):
			global_vars.append({'name':row[2], 'type':row[0]})
			c_row.append("&" + row[2])
		else:
			c_row.append(row[2])
		
		# generate table size entry
		if len(row[1]) > 0:
			c_row.append(row[1])
		else:
			c_row.append("sizeof(%s)" % row[0])

		# generate memory type entry
		c_row.append("MegaCAN::TableType_E::%s" % row[3])

		# generate flash offset entry
		c_row.append(table_flash_offset_str)

		c_table_rows.append(c_row)

	num_tables = len(table_list)
	c_table_defn_str = "#define NUM_TABLES %d\n" % num_tables
	c_table_defn_str += "static const MegaCAN::TableDescriptor_t TABLES[NUM_TABLES] = {\n"
	c_table_defn_str += make_padded_table(c_table_rows, indent=DEFAULT_INDENT, row_prefix="{", row_suffix="},") + "\n"
	c_table_defn_str += "};"

	now = datetime.now()
	dt_string = now.strftime("%Y/%m/%d %H:%M:%S")

	header_out_file.write("//============================================================\n")
	header_out_file.write("// this file was autogenerated by gen-ms-tables.py\n")
	header_out_file.write("// source file: %s\n" % args.csvfile.name)
	header_out_file.write("// date/time: %s\n" % dt_string)
	header_out_file.write("//============================================================\n")
	header_out_file.write('#pragma once\n')
	header_out_file.write('\n')
	header_out_file.write('#include <EEPROM.h>\n')
	header_out_file.write('#include <MegaCAN_ExtDevice.h>\n')
	header_out_file.write('#include <stdint.h>\n')

	header_out_file.write('\n')
	header_out_file.write(make_padded_table(table_size_defns, row_prefix="#define ", default_delim=" "))
	header_out_file.write('\n')
	header_out_file.write(make_padded_table(table_flash_offset_defns, row_prefix="#define ", default_delim=" "))

	header_out_file.write('\n\n')
	for idx, type in enumerate(nspace.types_by_name.values()):
		c_type_str = type.get_c_type_string()
		if idx > 0:
			header_out_file.write("\n\n")
		header_out_file.write(c_type_str)

	header_out_file.write('\n\n')
	header_out_file.write(c_table_defn_str)
	
def write_ini_file(args, nspace, **kwargs):
	indent = kwargs.get('indent', DEFAULT_INDENT)
	ini_out_file = args.ini_out_file
	if ini_out_file == None:
		ini_out_file = open("generated.ini", 'w')

	ini_out_str = args.ini_in_file.read()
	
	# start by cataloging the defined pages and their variables
	pages_by_id = {}
	out_pc = None
	gauge_vars_by_fullname = {}

	def has_gauge(type):
		return type.gauge_category != None and len(type.gauge_category) > 0

	def catalog_gauge_vars(type, full_type_name, offset):
		if has_gauge(type):
			gauge_vars_by_fullname[full_type_name] = type

	for type in nspace.types_by_name.values():
		page_id = type.row_obj.page_id
		if page_id == "OUT_PC":
			out_pc = type
			traverse_members(type, nspace, catalog_gauge_vars)
		elif len(page_id) > 0:
			page_id = int(page_id)
			pages_by_id[page_id] = type
			traverse_members(type, nspace, catalog_gauge_vars)

	for name, constant in nspace.constants_by_name.items():
		if has_gauge(constant):
			gauge_vars_by_fullname[name] = constant
	
	# --------------------------------------------------------
	# fill out gauge configurations

	gauge_var_fullnames_by_category = defaultdict(list)
	for fullname, var in gauge_vars_by_fullname.items():
		gauge_var_fullnames_by_category[var.gauge_category].append(fullname)

	category_idx = 0
	category_tables_str = ""
	for category, var_fullnames in gauge_var_fullnames_by_category.items():
		rows = []
		for fullname in var_fullnames:
			type = gauge_vars_by_fullname[fullname]
			row = []
			row.append(fullname + "Gauge")
			row.append(fullname)
			row.append('"%s"' % type.gauge_title)
			row.append('"%s"' % type.units)
			row.append('%0.6f' % type.lo)
			row.append('%0.6f' % type.hi)
			row.append('%0.6f' % type.lo_danger)
			row.append('%0.6f' % type.lo_warn)
			row.append('%0.6f' % type.hi_warn)
			row.append('%0.6f' % type.hi_danger)
			row.append('%d' % type.vd)
			row.append('%d' % type.ld)
			rows.append(row)

		COL_TITLES = ['Name', 'Var', 'Title', 'Units', 'Lo', 'Hi', 'LoD', 'LoW', 'HiW', 'HiD', 'vd', 'ld']
		str = indent + 'gaugeCategory = "%s"' % category + '\n'
		str += make_padded_table(rows, col_titles=COL_TITLES, indent=indent+DEFAULT_INDENT, delim_by_col={0:" = "})

		if category_idx > 0:
			category_tables_str += "\n\n"
		category_tables_str += str

		category_idx += 1
	
	ini_out_str = ini_out_str.replace('%%GAUGE_CONFIGURATIONS%%', category_tables_str)
	
	# --------------------------------------------------------
	# fill out number of pages
	n_pages = len(pages_by_id)
	ini_out_str = ini_out_str.replace('%%N_PAGES%%', "%d" % n_pages)

	# --------------------------------------------------------
	# fill out the page commands table
	page_cmd_rows = [
		["pageSize"],
		["pageIdentifier"],
		["burnCommand"],
		["pageReadCommand"],
		["pageValueWrite"],
		["pageChunkWrite"]]
	for page_id, page_type in pages_by_id.items():
		page_cmd_rows[0].append("128") # TODO compute page size
		page_cmd_rows[1].append('"\\$tsCanId\\x%02x"' % page_id)
		page_cmd_rows[2].append('"b\\$tsCanId\\x%02x"' % page_id)
		page_cmd_rows[3].append('"r%2i%2o%2c"')
		page_cmd_rows[4].append('"w%2i%2o%2c%v"')
		page_cmd_rows[5].append('"w%2i%2o%2c%v"')
	page_cmd_table = make_padded_table(page_cmd_rows, delim_by_col={0:" = "}, indent=indent)
	ini_out_str = ini_out_str.replace('%%PAGE_COMMANDS%%', page_cmd_table)

	constants_rows = []
	for constant in nspace.constants_by_name.values():
		row = [constant.name, '{ %s }' % constant.const_value, constant.code_comment]
		constants_rows.append(row)
	constants_table = make_padded_table(constants_rows, delim_by_col={0:" = ",1:"; "}, indent=indent)
	ini_out_str = ini_out_str.replace('%%CONSTANTS%%', constants_table)

	# --------------------------------------------------------
	# fill out the out_pc table definition
	if out_pc != None:
		page_defn = out_pc.get_ini_out_pc_string(nspace, indent=indent)
		ini_out_str = ini_out_str.replace('%%OUT_PC%%', page_defn)

	# --------------------------------------------------------
	# fill out all other table definitions
	page_defns_str = ""
	for idx, page_type in enumerate(pages_by_id.values()):
		if idx > 0:
			page_defns_str += "\n\n"
		page_defn = page_type.get_ini_page_string(nspace, indent=indent)
		page_defns_str += page_defn
	ini_out_str = ini_out_str.replace('%%PAGES%%', page_defns_str)

	ini_out_file.write(ini_out_str)

def main():
	parser = argparse.ArgumentParser(
		prog='gen-ms-tables',
		usage='%(prog)s [options]',
		description="generates C structures and TunerStudio *.ini files from a CSV source file")
	
	parser.add_argument(
		'csvfile',
		help="path to the source CSV to generate from",
		type=argparse.FileType('r'))
	
	parser.add_argument(
		'--ini-in-file',
		help="TunerStudio *.ini file template to populate",
		required=True, type=argparse.FileType('r'))
	
	parser.add_argument(
		'--ini-out-file',
		help="TunerStudio *.ini file to write output to",
		type=argparse.FileType('w'))
	
	parser.add_argument(
		'--header-out-file',
		help="generated header file path (default is '<csvfile_name>.h')",
		type=argparse.FileType('w'))

	parser.add_argument(
		'-v','--verbose',
		help="verbose logging level",
		action='count', default=0)

	args = parser.parse_args()
	verbose_level = args.verbose

	csvreader = csv.reader(args.csvfile, delimiter=',', quotechar='"')
	nspace = Namespace()
	table_list = []
	class ParseMode(Enum):
		TableDefns = 1
		TableListHeader = 2
		TableList = 3
	parse_mode = ParseMode.TableDefns
	for rowIdx, row in enumerate(csvreader):
		if verbose_level > 0:
			print('%4d: %s' % (rowIdx,', '.join(row)))

		if rowIdx == 0:
			continue # skip header row
		
		if parse_mode == ParseMode.TableDefns:
			if len(row) >= 1 and row[0] == 'TABLE_LIST':
				parse_mode = ParseMode.TableListHeader
			else:
				nspace.process_row(row)
		elif parse_mode == ParseMode.TableListHeader:
			parse_mode = ParseMode.TableList
		elif parse_mode == ParseMode.TableList:
			table_list.append(row)

	nspace.finalize()

	write_header_file(args, nspace, table_list)
	write_ini_file(args, nspace)

if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		pass

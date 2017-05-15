from numpy import *
from scipy.linalg import norm
import math
import time
import os

import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw

import utils.quaternions as quaternions
import utils.affine as affine
from utils.normalized import normalized_vec

import subprocess

import tempfile


__all__ = ["calc_frame_mercator"]

# R is the rotation matrix of the rotation of the object with respect to the laboratory frame, more precisely:
# if v are the components of a vector in the object frame, then R*v are its components in the laboratory frame 
# R is decomposed as 1) cc rot around z_object (yaw), followed by 2) cc rot around the
# new y_object (pitch), followed by 3) cc rot around the new new x_object (roll)
# yaw is the angle of 1) in range (-pi,pi]
# sin_pitch is the sine of the angle of 2)
# roll_rot_matrix is the 2D rotation matrix corresponding to the angle of 3)
def ypr_like_params(R): 
	yaw = math.atan2(R[1,0],R[0,0]) # (-pi,pi]
	sin_pitch = - R[2][0] #pitch grows downwards
	# the first column of the 2D rotation matrix corresponding to the roll angle
	#former:roll_unitvec = normalized(R[2,[2,1]])
	roll_unitvec = normalized_vec(R[2,[2,1]])
	# add the second column as the first column rotated by pi/2 degrees
	roll_rot_matrix = transpose(array([roll_unitvec,[-roll_unitvec[1],roll_unitvec[0] ] ]))
	return yaw,sin_pitch,roll_rot_matrix

def normalized_angle_hack(a):
	while(a>pi):
		a-=2*pi
	while(a<=-pi):
		a+=2*pi
	return a

def calc_frame_mercator_0(dims):
	frame_diagonal = norm(dims)
	tmp_frame_size = int(math.ceil(frame_diagonal))+2 # pixels needed for bilinear filtering: d=0 -> 2 px, d=0.001 -> 3 px
	tmp_frame = Image.new( 'RGB', (tmp_frame_size,)*2, "black") 
	def f(image,img_regions,rot_quaternion,frame_center_offset = array([0.0,0.0]),frame_rotation_offset = 0,azimuth_offset = 0):#todo use the rot offset
		R = quaternions.R_from_xyzw_quaternion(normalized_vec(rot_quaternion))
		yaw,sin_pitch,roll_matrix = ypr_like_params(R)
		yaw = normalized_angle_hack(yaw+azimuth_offset)

		# roll_matrix represents the rotation in the yz plane as seen from x>0 hemisphere,
		# but the user is in the other hemisphere, so he sees the opposite rotation. The computer graphics convention
		# has inverted the y axis, so in this convention roll_matrix actually represents the rotation as seen
		# by the user.
		# The following calculations use the computer graphics axes convention.

		c_ofs_tfmed = dot(roll_matrix, frame_center_offset)
		r = image.size[0]/(2*pi)
		
		# say zero yaw is in the middle of the image
		heading_mercator = r*(-yaw)+(image.size[0]-1)/2.0,(r/2.0)*math.log((1+sin_pitch)/(1-sin_pitch))+(image.size[1]-1)/2.0
		heading_mercator = array(heading_mercator)
		tmp_frame_center_in_img = heading_mercator + c_ofs_tfmed # this may leave the range of the image!!! todo: investigate whether this is a bug or not. replacing (-1,0,1) with something else in this case would probably solve the problem.
		tmp_frame_ul_in_img = floor(tmp_frame_center_in_img-frame_diagonal/2.0).astype(int)
		tmp_frame.paste("black")
		for s in (-1,0,1):
			tmp_frame.paste(image,tuple(-tmp_frame_ul_in_img+(s*image.size[0],0)))
		center = tmp_frame_center_in_img-tmp_frame_ul_in_img
		left_border = int(math.ceil(center[0]-image.size[0]/2.0))
		right_border = int(math.floor(center[0]+image.size[0]/2.0))
		tmp_frame.paste("black",(0,0,left_border,tmp_frame.size[1]))
		tmp_frame.paste("black",(right_border+1,0,tmp_frame.size[0],tmp_frame.size[1]))
		heading_mercator_in_tmp_frame = heading_mercator - tmp_frame_ul_in_img
		draw = ImageDraw.Draw(tmp_frame)
		#todo: make it scale according to mercator
		hrvf = 3.5 * pi/180*r # half of real view field
		cs = 0.3
		draw.line(tuple(heading_mercator_in_tmp_frame-[0,hrvf*cs])+tuple(heading_mercator_in_tmp_frame+[0,hrvf*cs]),fill="red",width = 2)
		draw.line(tuple(heading_mercator_in_tmp_frame-[hrvf*cs,0])+tuple(heading_mercator_in_tmp_frame+[hrvf*cs,0]),fill="red",width = 2)
		for _delta in range(4):
			hrvf2 = hrvf+_delta
			draw.ellipse(tuple(heading_mercator_in_tmp_frame-hrvf2)+tuple(heading_mercator_in_tmp_frame+hrvf2),outline = "red") 
		pic_affine_tfm = reduce(dot, [
			affine.translation_as_affine(center),
			affine.linear_as_affine(roll_matrix),
			affine.translation_as_affine( ( -dims[0]/2.0, -dims[1]/2.0 ) ),
		])
		result_frame = tmp_frame.transform(dims,Image.AFFINE,pic_affine_tfm[0:2,0:3].flatten(),Image.BILINEAR)
		result_region_pixel = (0,0,0,0)
		heading_merc_int = heading_mercator.astype(int)#consider rounding it instead
		if(0<=heading_merc_int[0]<img_regions.size[0]):
			if(0<=heading_merc_int[1]<img_regions.size[1]):
				result_region_pixel = img_regions.getpixel(tuple(heading_mercator))
		return result_frame,result_region_pixel
	return f

def convert_svg_to_pixmap(svg_string):
	converter_args = ["rsvg-convert"]+["/dev/stdin"]
	with tempfile.TemporaryFile() as fp:
		converter = subprocess.Popen(converter_args,stdin = subprocess.PIPE,stdout=fp, shell = False)
		converter.communicate(svg_string)
		fp.seek(0)
		pixmap= Image.open(fp)
		pixmap.load()
		return pixmap


def get_svg_with_path_attributes(filename,thickness,color):
	svg_tree = ET.parse(filename)
	SVG_NS = "http://www.w3.org/2000/svg"
	svg_root = svg_tree.getroot()
	for p in svg_root.findall("{%s}path"%SVG_NS):
		p.attrib["stroke"] = color
		p.attrib["stroke-width"] = str(thickness)
	return ET.tostring(svg_root)

def get_boundaries_pixmap(filename_regions,svg_line_thickness,color):
	svg_boundaries=get_svg_with_path_attributes(filename_regions,svg_line_thickness,color)
	return convert_svg_to_pixmap(svg_boundaries)

def uniquely_colorize_paths_in_svg(filename):
	svg_tree = ET.parse(filename)
	SVG_NS = "http://www.w3.org/2000/svg"
	svg_root = svg_tree.getroot()
	color_id_map = {}
	for n,p in enumerate(svg_root.findall("{%s}path"%SVG_NS)):
		del p.attrib["stroke"]
		del p.attrib["stroke-width"]
		color = (n+1)
		p.attrib["fill"] = "#"+format(color,'06x') # better reserve black for special purposes
		color_id_map[(0,0,color)] = p.attrib["id"]
	return ET.tostring(svg_root),color_id_map

def get_interiors_pixmap(filename_regions):
	svg_regions, color_id_map = uniquely_colorize_paths_in_svg(filename_regions)
	return convert_svg_to_pixmap(svg_regions),color_id_map

def get_background_and_regions(filename_panorama, filename_regions, final_image_width = None): # todo: do not thread the filenames through so many calls! do dependency injection!
	img = Image.open(filename_panorama)
	if(final_image_width==None):
		final_image_width = img.size[0]
	final_line_thickness = 3.0
	svg_line_thickness = final_line_thickness * img.size[0]/final_image_width
	boundaries_pixmap = get_boundaries_pixmap(filename_regions,svg_line_thickness,"yellow")
	img.paste(boundaries_pixmap,(0,0),boundaries_pixmap)
	
	interiors_pixmap, color_id_map = get_interiors_pixmap(filename_regions)
	
	if(img.size!=interiors_pixmap.size):
		print img.size
		print interiors_pixmap.size
		raise BaseException("background and regions images' sizes do not match")
	if(img.size[0] != final_image_width): #todo: consider resizing sooner to reduce memory peak
		img = img.resize((int(final_image_width), int(img.size[1]*final_image_width/img.size[0])),Image.ANTIALIAS)
		interiors_pixmap = interiors_pixmap.resize((int(final_image_width), int(interiors_pixmap.size[1]*final_image_width/interiors_pixmap.size[0])),Image.NEAREST)
	return img,interiors_pixmap,color_id_map
	
def calc_frame_mercator(dims, center, filename_panorama, filename_regions, relative_zoom = None, azimuth_offset = 0):  #dims = (width,height)
	final_image_width = img.size[0] if relative_zoom==None else relative_zoom*dims[0]
	img,img_regions,color_id_map = get_background_and_regions(filename_panorama, filename_regions,final_image_width)
	cfm = calc_frame_mercator_0(dims)
	def f(rot_quaternion):
		frame,target = cfm(img,img_regions,rot_quaternion,center,azimuth_offset = azimuth_offset)
		return array(frame),color_id_map.get((target[0],target[1],target[2]), None)
	return f

def calc_frame_mercator_and_play_sound(cfm,directory,language,delay):
	last_change = {"target":None,"time":time.time(),"fired":False}
	def f(rot_quaternion):
		frame,target_id = cfm(rot_quaternion)
		if(target_id!=last_change["target"]):
			last_change.update({"target":target_id,"time":time.time(),"fired":False})
		else:
			if((not last_change["fired"]) and time.time()-last_change["time"]>delay):
				last_change["fired"]=True
				if(target_id!=None):
					filename = "\""+directory+target_id+"/languages/"+language+"/name/voice.wav"+"\""
					print filename
					os.system("afplay "+filename+" &")	
		return frame
	return f

def demo(content_dir,project_name,panorama_name,image_name,language,delay,relative_zoom):
	dir_project = content_dir+ "/projects/"+project_name
	dir_pano = dir_project+"/panoramas/"+panorama_name
	filename_panorama = dir_pano+"/images/"+image_name+"/image.jpg"
	filename_regions = dir_pano+"/regions/image.svg" # pozor !!! tohle je dobudoucna uplne blbe, nezohlednuje to zadne ty rotvecy !!!
	cfm = calc_frame_mercator((320,240),(0,0),filename_panorama,filename_regions,relative_zoom)
	cfmps = calc_frame_mercator_and_play_sound(cfm,dir_project+"/points of interest/",language,delay)
	return cfmps

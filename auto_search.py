from PhPy import phGetPh,   phDoPh,      phSetPh
from PhPy import phGetCam,  phDoCam,     phSetCam
from PhPy import phGetCine, phDoCine,    phSetCine
import sys

from pipython import GCSDevice, pitools

import numpy as np
from numpy import linalg as LA
from tensorflow.keras.models import load_model

import time
import tkinter as tk
import os
import glob


PhantomDictionary = phGetPh()
globals().update(PhantomDictionary)
print("\nPython version", sys.version, "\n\n")


def FindCameras():
    camcnt = phGetPh(CameraCount)

    if camcnt == 0:  # No real cannera connected,  add a simulated camera
        phDoPh(AddSimulatedCamera, (12002, 1))  # hardware version; 12002 = VEO 1310L
        phDoPh(AddSimulatedCamera, (8001, 0))  # Miro 310
        phDoPh(AddSimulatedCamera, (122, 0))  # v710 mono
        phDoPh(AddSimulatedCamera, (4001, 1))  # Ph 4k color
        camcnt = phGetPh(CameraCount)
    if camcnt == 1:
        print("List of ", camcnt, " Phantom camera connected to this computer\n")
    else:
        print("List of ", camcnt, " Phantom cameras connected to this computer\n")

    # Display index, serial and name of available cameras
    dash = '-' * 55
    print(dash)
    print("cn  Serial Version  Name            Model")
    print(dash)
    for cn in range(camcnt):
        Serial1 = phGetCam(Serial, cn)
        Name1 = phGetCam(Name, cn)
        Version1 = phGetCam(HardwareVersion, cn)
        Model1 = phGetCam(Model, cn)
        print('{:>2d}  {:>6d}   {:>5d}  {:<15s} {:<15s}'.format(cn, Serial1, Version1, Name1, Model1))
    print("\n")

    offl = phGetCam(Offline, 0)
    print("Offline cam 0:", offl)
    return camcnt


class AutoSearch(tk.Frame):

    def __init__(self, root, c867, e518, area, step, criteria, file_path, model_lateral,
                 model_quality, model_axial_coarse, model_axial_fine):
        
        self.master = root
        self.c867 = c867
        self.e518 = e518
        self.area = area
        self.step = step
        self.criteria = criteria
        self.file_path = file_path
        self.model_lateral = model_lateral
        self.model_quality = model_quality
        self.model_axial_coarse = model_axial_coarse
        self.model_axial_fine = model_axial_fine

        self.lateral_scan()

    @staticmethod
    def last_num(file_dir, file_type):
        file_list = glob.glob(os.path.join(file_dir, f"*.{file_type}"))
        seq_list = []
        dic_file = {}
        try:
            for r in range(len(file_list)):
                dic_file[file_list[r]] = int(file_list[r].split("\\")[-1].split(".")[0].split('_')[-1])
            file_directory = sorted(dic_file.items(), key=lambda x: x[1])
            for file in file_directory:
                seq_list.append(file[1])
            return int(seq_list[-1]) + 1
        except:
            return 1

    def ph_save(self):
        def ProgressIndicator(hcine, percent):
            #print("Cine save: hcine = ", hcine, "percent = ", percent,
            #      "%  =======================================================")
            if percent == 100:
                phDoCine(Close, hcine)
            return 1
        filename = self.file_path
        cn = 0
        cur_cine = phGetCam(ActivePartition, cn)[0]
        if cur_cine == 0:
            phDoCam(Record, cn)
            cur_cine += 1
        phDoCam(Trigger, cn)
        while phGetCam(Recorded, cn, cur_cine) != 1:
            pass
        hcine = phGetCam(CineHandle, cn, cur_cine)
        if filename.find('.') != -1:
            filename = filename.split('.')[0]
        result_dir = os.path.dirname(filename)
        file_name = os.path.basename(filename)
        file_index = self.last_num(result_dir, 'cine')
        file_path = os.path.join(result_dir, f'{file_name}_{file_index}.cine')
        phSetCine(SaveName, hcine, file_path)
        phSetCine(SaveType, hcine, svv_RawCine)
        (f, l) = phGetCine(Range, hcine)
        phSetCine(SaveRange, hcine, (f, l))
        phSetPh(Callback0, ProgressIndicator)
        phDoCine(Save, hcine)

    def lateral_scan(self):
        mox = self.c867.axes[0]
        moy = self.c867.axes[1]
        self.c867.VEL(mox, 0.05)
        self.c867.VEL(moy, 0.05)
        motorized_init_pos = [self.c867.qPOS(mox)[mox], self.c867.qPOS(moy)[moy]]
        scan_list = self.set_motorized_scan(motorized_init_pos, self.area, self.step) # step = 0.04
        x = self.e518.axes[1]
        y = self.e518.axes[0]
        z = self.e518.axes[2]
        piezo_init_pos = [self.e518.qPOS(x)[x], self.e518.qPOS(y)[y], self.e518.qPOS(z)[z]]
        nb_points = len(scan_list)
        i = 0
        while i <= nb_points:
            if i == nb_points - 1:
                self.c867.MOV(mox, scan_list[-1][0])
                self.c867.MOV(moy, scan_list[-1][1])
                pitools.waitontarget(self.c867, axes=[mox, moy])
                break
            else:
                self.c867.MOV(mox, scan_list[i][0])
                self.c867.MOV(moy, scan_list[i][1])
                pitools.waitontarget(self.c867, axes=[mox, moy])
                self.e518.MOV(z, piezo_init_pos[2])
                pitools.waitontarget(self.e518)
                i += 1
                is_save = self.centralize(self.model_lateral, self.e518, 0.5, self.model_quality)
                # centralize the position
                if is_save:
                    self.auto_focus(self.model_axial_coarse, self.model_axial_fine, self.e518)
                    self.c867.SVO(mox, False)
                    self.c867.SVO(moy, False)
                    self.ph_save()
                    self.c867.SVO(mox, True)
                    self.c867.SVO(moy, True)
                else:
                    self.e518.MOV(x, piezo_init_pos[0])
                    self.e518.MOV(y, piezo_init_pos[1])
                    self.e518.MOV(z, piezo_init_pos[2])
                    pitools.waitontarget(self.e518)

    @staticmethod
    def set_motorized_scan(motorized_init_pos, scan_range, interval_pos):
        init_x = motorized_init_pos[0]
        init_y = motorized_init_pos[1]
        decimal = len(str(interval_pos)[str(interval_pos).find('.'):]) + 2
        nb_points = int(((scan_range // interval_pos) + 1) ** 2)
        stop_length = int(scan_range / interval_pos)
        if stop_length % 2 == 0:
            stop_point = (np.around(-interval_pos * (stop_length / 2) + init_x, decimals=decimal),
                          np.around(interval_pos * (stop_length / 2) + init_y, decimals=decimal))
        else:
            left_space = stop_length // 2 + 1
            stop_point = (np.around(interval_pos * left_space + init_x, decimals=decimal),
                          np.around(-interval_pos * (stop_length - left_space) + init_y, decimals=decimal))
        motorized_scan_x = [init_x]
        motorized_scan_y = [init_y]
        direction_dict = {
            0: "up",
            1: "right",
            2: "down",
            3: "left"
        }
        direction = 0
        for pos_group in range(2, nb_points + 1, 2):
            dir_group = int(pos_group / 2)
            for increment in range(pos_group):
                if increment == dir_group:
                    direction += 1
                    if direction == 4:
                        direction = 0
                action = direction_dict[direction]
                if action == "up":
                    motorized_init_pos[1] += interval_pos
                elif action == "right":
                    motorized_init_pos[0] += interval_pos
                elif action == "down":
                    motorized_init_pos[1] -= interval_pos
                elif action == "left":
                    motorized_init_pos[0] -= interval_pos
                motorized_scan_x.append(np.around(motorized_init_pos[0], decimals=decimal))
                motorized_scan_y.append(np.around(motorized_init_pos[1], decimals=decimal))
                if (motorized_scan_x[-1], motorized_scan_y[-1]) == stop_point:
                    motorized_scan_x.append(init_x)
                    motorized_scan_y.append(init_y)
                    motor_position_list = list(zip(motorized_scan_x, motorized_scan_y))
                    return motor_position_list
            direction += 1
            if direction == 4:
                direction = 0

    def centralize(self, model_lateral, e518, edge_value, model_quality):
        x = e518.axes[1]
        y = e518.axes[0]
        z = e518.axes[2]
        operation = 1
        for attempt in range(5):
            current_pos = [e518.qPOS(x)[x], e518.qPOS(y)[y], e518.qPOS(z)[z]]
            hcine = phGetCam(CineHandle, 0, -1)
            image = phGetCine(Image_np, hcine, (0, 0, gci_LeftAlign))[0, 0:640, :]
            phDoCine(Close, hcine)
            image = self.img_preprocessing(arr=image, nor=65535, new_shape=(128, 128))
            mask_img = model_lateral.predict(image.reshape((1, 128, 128, 1)))[0, :, :, 0]
            if attempt < 3:
                center = self.find_center(mask_img, edge_value)
            else:
                if operation > 3:
                    center = self.find_edge(mask_img, edge_value)
                else:
                    center = None
            if center is not None:
                operation += 1
                mod_x = int(center[1] - 63) * 0.04 * 5
                mod_y = int(center[0] - 63) * 0.04 * 5
                try:
                    e518.MOV(x, current_pos[0] + mod_x)
                    e518.MOV(y, current_pos[1] + mod_y)
                    e518.MOV(z, current_pos[2])
                    pitools.waitontarget(e518)
                except:
                    e518.MOV(x, 100)
                    e518.MOV(y, 100)
                    pitools.waitontarget(e518)
                    return None
        if operation == 1:
            return None
        else:
            hcine = phGetCam(CineHandle, 0, -1)
            image_qc = phGetCine(Image_np, hcine, (0, 0, gci_LeftAlign))[0, 0:640, :]
            phDoCine(Close, hcine)
            image = self.img_preprocessing(arr=image_qc, nor=65535, new_shape=(320, 320))
            quality = model_quality.predict(image.reshape((1, 320, 320, 1)))[0][0]
            if quality > self.criteria:
                return True
            else:
                return None

    def img_preprocessing(self, arr, nor=None, new_shape=None):
        if nor is not None:
            arr = arr / nor
            if new_shape is not None:
                if len(arr.shape) == 3:
                    arr_out = np.ndarray((arr.shape[0], new_shape[0], new_shape[1]))
                    for i in range(arr.shape[0]):
                        arr_out[i, :, :] = self.binning(arr[i, :, :], new_shape)
                else:
                    arr_out = self.binning(arr, new_shape)
            else:
                arr_out = arr
        else:
            pass
        return arr_out

    @staticmethod
    def binning(arr, new_shape):
        shape = (new_shape[0], arr.shape[0] // new_shape[0],
                 new_shape[1], arr.shape[1] // new_shape[1])
        return arr.reshape(shape).mean(-1).mean(1)

    def find_center(self, img, edge_value):
        portion = 0
        largest = 0
        center = []
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if img[i, j] > edge_value:
                    portion += 1
                    pos = [i, j]
                    distance = self.search_min(img, pos, edge_value)
                    if distance > largest:
                        largest = distance
                        center = pos
        if portion > 1000:
            return center
        else:
            return None

    def find_edge(self, image, edge_value):
        center = self.find_center(image, edge_value)
        edge = {}
        try:
            arr_vertical = image[:, center[1]]
            arr_horizontal = image[center[0], :]
            for i in range(len(arr_vertical)):
                try:
                    up = arr_vertical[center[0] - i]
                    down = arr_vertical[center[0] + i]
                    left = arr_horizontal[center[1] - i]
                    right = arr_horizontal[center[1] + i]
                    if up < 0.5:
                        if 'up' not in edge:
                            edge['up'] = center[0] - i + 1
                    if down < 0.5:
                        if 'down' not in edge:
                            edge['down'] = 128 - (center[0] + i - 1)
                    if left < 0.5:
                        if 'left' not in edge:
                            edge['left'] = center[1] - i + 1
                    if right < 0.5:
                        if 'right' not in edge:
                            edge['right'] = 128 - (center[1] + i - 1)
                except IndexError:
                    pass
            vertical_correction = edge['up'] - edge['down']
            horizontal_correction = edge['left'] - edge['right']
            return [vertical_correction + 63, horizontal_correction + 63]
        except:
            return None

    def search_min(self, image, cur_loc, edge_value):
        space = 10
        img_x = image.shape[0]
        img_y = image.shape[1]
        distance = None
        while distance is None:
            x1 = self.get_valid_index(cur_loc[0] - space, img_x)
            x2 = self.get_valid_index(cur_loc[0] + space, img_x)
            y1 = self.get_valid_index(cur_loc[1] - space, img_y)
            y2 = self.get_valid_index(cur_loc[1] + space, img_y)
            img = image[x1:x2, y1:y2]
            distance = self.find_min_distance(img, cur_loc, edge_value)
            space += 5
        return distance

    @staticmethod
    def get_valid_index(x, shape):
        if x > shape:
            return shape + 1
        elif x < 0:
            return 0
        else:
            return x

    @staticmethod
    def find_min_distance(image, cen_loc, edge_value):
        indices = np.argwhere(image < edge_value)
        if len(indices) == 0:
            return None
        else:
            m = float('inf')
            for index in indices:
                index = np.array([index[0] + cen_loc[0] - 1, index[1] + cen_loc[1] - 1])
                distance = LA.norm((cen_loc - index), 2)
                m = min(m, distance)
            return m

    def auto_focus(self, model_coarse, model_fine, e518):
        z = e518.axes[2]
        cur_z = e518.qPOS(z)[z]
        self.coarse_mod(z, cur_z, e518, model_coarse)
        cur_z = e518.qPOS(z)[z]
        self.fine_mod(z, cur_z, e518, model_fine)

    def coarse_mod(self, z, cur_z, e518, model_coarse):
        image_coarse = np.ndarray((3, 640, 640))
        coarse_moving_list = [-1, 0, 1]
        for i in coarse_moving_list:
            e518.MOV(z, cur_z + i)
            pitools.waitontarget(e518, axes=z)
            hcine = phGetCam(CineHandle, 0, -1)
            image_coarse[i + 1, :, :] = phGetCine(Image_np, hcine, (0, 0, gci_LeftAlign))[0, 0:640, :]
            phDoCine(Close, hcine)
        imgs_coarse = self.img_preprocessing(arr=image_coarse, nor=65535, new_shape=(320, 320))[:, 32:288, 32:288]
        imgs_coarse = np.reshape(imgs_coarse, (1, 256, 256, 3))
        c1, c2, c3 = model_coarse.predict(imgs_coarse)[0]
        coarse_key_frame = [c1, c2, c3]
        coarse_predicted_z = coarse_moving_list[coarse_key_frame.index(max(coarse_key_frame))]
        e518.MOV(z, cur_z + coarse_predicted_z)
        pitools.waitontarget(e518)
        # coarse modification

    def fine_mod(self, z, cur_z, e518, model_fine):
        image_fine = np.ndarray((7, 640, 640))
        fine_moving_list = [-0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75]
        for j in range(len(fine_moving_list)):
            e518.MOV(z, cur_z + fine_moving_list[j])
            pitools.waitontarget(e518, axes=z)
            hcine = phGetCam(CineHandle, 0, -1)
            image_fine[j, :, :] = phGetCine(Image_np, hcine, (0, 0, gci_LeftAlign))[0, 0:640, :]
            phDoCine(Close, hcine)
        image_fine = self.img_preprocessing(arr=image_fine, nor=65535, new_shape=(320, 320))[:, 32:288, 32:288]
        image_fine = np.reshape(image_fine, (1, 256, 256, 7))
        f1, f2, f3, f4, f5, f6, f7 = model_fine.predict(image_fine)[0]
        fine_key_frame = [f1, f2, f3, f4, f5, f6, f7]
        # print(f'fine predicted result: {fine_key_frame}')
        fine_predicted_z = fine_moving_list[fine_key_frame.index(max(fine_key_frame))]
        e518.MOV(z, cur_z + fine_predicted_z)
        # print(cur_z + fine_predicted_z, fine_predicted_z)
        pitools.waitontarget(e518)
        # fine modification


class JoyStick(tk.Frame):

    def __init__(self, root, stage):
        self.stage = stage
        self.x = self.stage.axes[1]
        self.y = self.stage.axes[0]
        self.x_range = [self.stage.qTMN(self.x)[self.x], self.stage.qTMX(self.x)[self.x]]
        self.y_range = [self.stage.qTMN(self.y)[self.y], self.stage.qTMX(self.y)[self.y]]
        self.nb_axis = len(self.stage.axes)
        if self.nb_axis == 3:
            self.z = self.stage.axes[2]
        if self.stage.HasHIN():
            self.stage.HIN(self.x, False)
            self.stage.HIN(self.y, False)

        self.popup_win = tk.Toplevel(root)
        dev = self.stage.devname.split('.')[0]
        self.popup_win.wm_title(f"{dev}  Controller")
        self.popup_win.geometry('320x320')
        self.popup_win.configure(bg='white')
        self.popup_win.resizable(0, 0)
        self.popup_win.update()
        self.wn_size = self.popup_win.winfo_width()
        self.wn_pos = [self.popup_win.winfo_rootx(), self.popup_win.winfo_rooty()]
        self.radius = 110
        pad = 200
        self.canvas_range = tk.Canvas(self.popup_win,
                                      bg='white',
                                      borderwidth=0,
                                      highlightthickness=0)
        range_pos = (self.wn_size - (self.radius + pad)) * 0.5
        range_size = (self.radius + pad)
        relsize = range_size / self.wn_size
        self.canvas_range.place(x=range_pos, y=range_pos, relwidth=relsize, relheight=relsize)
        self.create_circle(range_size // 2, range_size // 2, self.radius, self.canvas_range, None)

        if self.nb_axis == 3:
            self.canvas_range.bind('<Enter>', self.bound_to_mousewheel)
            self.canvas_range.bind('<Leave>', self.unbound_to_mousewheel)

        self.dot = tk.Canvas(self.canvas_range,
                             bg='white',
                             borderwidth=0,
                             highlightthickness=0)
        size_ratio = 3
        dot_size = range_size / size_ratio
        self.dot_pos = (range_size - dot_size) * 0.5
        self.dot.place(x=self.dot_pos, y=self.dot_pos, relwidth=1 / size_ratio, relheight=1 / size_ratio)
        self.create_circle(dot_size // 2, dot_size // 2, dot_size * 0.8 // 2, self.dot, 'black')
        self.dot.bind("<Motion>", self.mouse_appearance)
        self.dot.bind("<B1-Motion>", self.drag)
        self.dot.bind("<ButtonRelease-1>", self.centralize)
        self.offset = dot_size - range_pos - dot_size * 0.2 * 2

        self.generator = 0
        self.pressed = False
        self.increment_x = 0
        self.increment_y = 0

    @staticmethod
    def create_circle(x, y, r, canvas, fill):  # center coordinates, radius
        x0 = x - r
        y0 = y - r
        x1 = x + r
        y1 = y + r
        return canvas.create_oval(x0, y0, x1, y1, outline='black', width=2, fill=fill)

    def bound_to_mousewheel(self, event):
        self.canvas_range.bind("<MouseWheel>", self.set_piezo_axis)

    def unbound_to_mousewheel(self, event):
        self.canvas_range.unbind("<MouseWheel>")

    def drag(self, event):
        self.pressed = True
        if self.generator == 0:
            self.set_stage()
        cur_wn_pos = [self.popup_win.winfo_rootx(), self.popup_win.winfo_rooty()]
        if cur_wn_pos != self.wn_pos:
            self.wn_pos = cur_wn_pos
        x = event.widget.winfo_pointerx() - self.wn_pos[0] - self.offset
        y = event.widget.winfo_pointery() - self.wn_pos[1] - self.offset
        x, y = self.get_coord(x, y)
        event.widget.place(x=x, y=y)

    def mouse_appearance(self, event):
        self.dot.config(cursor="hand2")

    def centralize(self, event):
        self.pressed = False
        self.dot.place(x=self.dot_pos, y=self.dot_pos)

    def get_coord(self, x, y):
        delta_x = self.dot_pos - x
        delta_y = self.dot_pos - y
        radius = (delta_x ** 2 + delta_y ** 2) ** 0.5
        ratio = radius / self.radius
        if ratio <= 1:
            self.increment_x = (x - self.dot_pos) / self.radius
            self.increment_y = (self.dot_pos - y) / self.radius
            return x, y
        else:
            if delta_x < 0:
                edge_x = abs(delta_x / ratio) + self.dot_pos
            else:
                edge_x = self.dot_pos - (delta_x / ratio)
            if delta_y < 0:
                edge_y = abs(delta_y / ratio) + self.dot_pos
            else:
                edge_y = self.dot_pos - (delta_y / ratio)
            self.increment_x = (edge_x - self.dot_pos) / self.radius
            self.increment_y = (self.dot_pos - edge_y) / self.radius
            return edge_x, edge_y

    def set_stage(self):
        cur_pos = [self.stage.qPOS(self.x)[self.x],
                   self.stage.qPOS(self.y)[self.y]]
        self.generator = 0
        if self.pressed:
            if self.nb_axis == 3:
                increment_x = -self.increment_x
                increment_y = -self.increment_y
                unit_x = unit_y = self.stage.qVEL(self.x)[self.x] * 4
            else:
                increment_x = self.increment_x
                increment_y = self.increment_y
                unit_x = 0.02 * (1 + abs(increment_x))
                unit_y = 0.02 * (1 + abs(increment_y))
                self.stage.VEL(self.x, unit_x * abs(increment_x))
                self.stage.VEL(self.y, unit_y * abs(increment_y))
            target_x = cur_pos[0] + unit_x * increment_x
            target_y = cur_pos[1] + unit_y * increment_y
            if self.x_range[0] < target_x < self.x_range[1]:
                self.stage.MOV(self.x, target_x)
            if self.y_range[0] < target_y < self.y_range[1]:
                self.stage.MOV(self.y, target_y)
            self.generator = root.after(10, self.set_stage)
        else:
            if self.generator != 0:
                root.after_cancel(self.generator)

    def set_piezo_axis(self, event):
        cur_z = self.stage.qPOS(self.z)[self.z]
        target_z = cur_z + event.delta / 1200
        self.stage.MOV(self.z, target_z)


class Main(tk.Frame):

    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        FindCameras()
        
        popup_win = tk.Toplevel(root)
        popup_win.wm_title("JoyStick Control")
        popup_win.geometry('450x300')
        popup_win.configure(bg='white')
        popup_win.resizable(0, 0)

        sn_label_1 = tk.Label(popup_win, text='C867 Serial Number:',
                              bg='white', font='Arial 12', )
        sn_label_1.place(relx=0.05, rely=0.1)
        self.sn_entry_1 = tk.Entry(popup_win, width=20)
        self.sn_entry_1.insert(0, '0120027194')
        self.sn_entry_1.place(relx=0.42, rely=0.115)

        sn_label_2 = tk.Label(popup_win, text='E518 Serial Number:',
                              bg='white', font='Arial 12', )
        sn_label_2.place(relx=0.05, rely=0.3)
        self.sn_entry_2 = tk.Entry(popup_win, width=20)
        self.sn_entry_2.insert(0, '120027848')
        self.sn_entry_2.place(relx=0.42, rely=0.315)

        self.enter_btn = tk.Button(popup_win, text='Enter',
                              font='Arial 12', width=12,
                              command=self.get_enter)
        self.enter_btn.place(relx=0.7, rely=0.85)

        area_label = tk.Label(popup_win, text='Search area (um):', font='Arial 12')
        area_label.place(relx=0.05, rely=0.5)
        self.area = tk.StringVar()
        self.area_entry = tk.Entry(popup_win, width=5, textvariable=self.area)
        self.area_entry.place(relx=0.37, rely=0.515)

        step_label = tk.Label(popup_win, text='Search step (um):', font='Arial 12')
        step_label.place(relx=0.5, rely=0.5)
        self.step = tk.StringVar(value=0.04)
        self.step_entry = tk.Entry(popup_win, width=5, textvariable=self.step)
        self.step_entry.place(relx=0.82, rely=0.515)

        file_path = tk.Label(popup_win, text='File path:', font='Arial 12')
        file_path.place(relx=0.05, rely=0.7)
        self.file_path = tk.Entry(popup_win, width=55)
        self.file_path.place(relx=0.23, rely=0.715)

        criteria = tk.Label(popup_win, text='Criteria (0~1):', font='Arial 12')
        criteria.place(relx=0.05, rely=0.85)
        self.criteria = tk.Entry(popup_win, width=5)
        self.criteria.place(relx=0.3, rely=0.865)

        self.model_lateral = []
        self.model_quality = []
        self.model_axial_coarse = []
        self.model_axial_fine = []

        self.c867 = ''
        self.e518 = ''

        popup_win.protocol("WM_DELETE_WINDOW", lambda e: root.destroy())

    def get_enter(self):
        if self.enter_btn['text'] == 'Enter':
            self.get_joystick()
        else:
            self.set_search()

    def get_joystick(self):
        sn_1 = self.sn_entry_1.get()
        sn_2 = self.sn_entry_2.get()
        self.c867 = GCSDevice()
        self.c867.ConnectUSB(serialnum=sn_1)
        print('connected: {}'.format(self.c867.qIDN().strip()))
        pitools.startup(self.c867, stages=None, refmodes=None)
        JoyStick(root, self.c867)

        self.e518 = GCSDevice()
        self.e518.ConnectUSB(serialnum=sn_2)
        print('connected: {}'.format( self.e518.qIDN().strip()))
        pitools.startup( self.e518, stages=None, refmodes=None)
        JoyStick(root,  self.e518)
        self.enter_btn['text'] = 'Start'

    def set_search(self):
        self.apply_machine()
        AutoSearch(root, self.c867, self.e518, float(self.area.get()), float(self.step.get()), float(self.criteria.get()), self.file_path.get(),
                   self.model_lateral, self.model_quality, self.model_axial_coarse, self.model_axial_fine)

    def apply_machine(self):
        model_lateral_path = r'C:\Users\NBP\Desktop\save_test\model\lateral\model_lateral.hdf5'
        model_quality_control_path = r'C:\Users\NBP\Desktop\save_test\model\quality\model_quality_control.hdf5'
        model_axial_coarse_path = r'C:\Users\NBP\Desktop\save_test\model\axial_coarse\model_axial_coarse.hdf5'
        model_axial_fine_path = r'C:\Users\NBP\Desktop\save_test\model\aixal_fine\model_axial_fine.hdf5'
        self.model_lateral = load_model(model_lateral_path)
        self.model_quality = load_model(model_quality_control_path)
        self.model_axial_coarse = load_model(model_axial_coarse_path)
        self.model_axial_fine = load_model(model_axial_fine_path)
        print('models successfully loaded')


if __name__ == '__main__':
    root = tk.Tk()
    root.iconify()
    Main(root)
    root.mainloop()
import copy
import logging
import os
import tkinter as tk
import tkinter.filedialog as filedialog
import tkinter.ttk as ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import geometry.complex as cm
import geometry.parser as gp
import geometry.spatial_manipulation as spm
import project_manager as pm
import slicer.slice_manager as sm
import slicer.wire_cutter as wc
from util.util_functions import is_float
from .styles import PrimaryStyle


class WindowState(object):
    HOME = 0
    MACHINE_SETUP = 1
    WING = 2
    CAD = 3
    DATABASE = 4
    RANGE = range(0, 5)


class ToggleButton(tk.Frame):
    def __init__(self, master, top_level_scroll, index, width, height, text, bg, fg, icon=None):
        super(ToggleButton, self).__init__(master, width=width, height=height)

        self.index = index
        self.top_level_scroll = top_level_scroll
        self.grid_propagate(False)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self.button = tk.Button(self, text=text, icon=icon, bg=bg, fg=fg)
        self.button.grid(row=0, column=0, sticky=tk.NSEW)
        self.button.config(command=lambda: self.top_level_scroll.select(self.index))


class ScrollableSelectionMenu(tk.Frame):

    def __init__(self, master, root_context):
        super(ScrollableSelectionMenu, self).__init__(master)
        self.curr_select = None
        self.items = list()
        self.root = root_context

        self.grid_rowconfigure(index=0, weight=1)
        self.grid_rowconfigure(index=1, weight=50)
        self.grid_rowconfigure(index=2, weight=1)
        self.grid_columnconfigure(index=0, weight=12)
        self.grid_columnconfigure(index=1, weight=1)
        self.grid_propagate(False)
        self.pack_propagate(False)

        self.add_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.add_frame.grid(row=0, column=0, sticky=tk.NSEW, columnspan=2)
        self.add_frame.grid_rowconfigure(index=0, weight=1)
        self.add_frame.grid_columnconfigure(index=0, weight=1)

        self.add_button = tk.Button(self.add_frame, text='Add', command=lambda: self._add_selection(),
                                    background=PrimaryStyle.PRIMARY_COLOR,
                                    fg=PrimaryStyle.FONT_COLOR)
        self.add_button.grid(row=0, column=0, sticky=tk.NSEW)

        self.canvas_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                     highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                     highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.canvas_frame.grid(row=1, column=0, sticky=tk.NSEW, columnspan=2)
        self.canvas_frame.grid_rowconfigure(index=0, weight=1)
        self.canvas_frame.grid_columnconfigure(index=0, weight=29)
        self.canvas_frame.grid_columnconfigure(index=1, weight=1)
        self.canvas_frame.grid_propagate(False)
        self.canvas_frame.pack_propagate(False)

        self.primary_canvas = tk.Canvas(self.canvas_frame,
                                        background=PrimaryStyle.PRIMARY_COLOR, width=65)
        self.primary_canvas.grid(row=0, column=0, sticky=tk.NSEW)
        self.primary_canvas.pack_propagate(False)
        self.primary_canvas.grid_propagate(False)

        self.scroll_window = tk.Frame(self.primary_canvas,
                                      background=PrimaryStyle.PRIMARY_COLOR)

        self.primary_canvas.create_window(0, 0, window=self.scroll_window, anchor=tk.NW)

        self.v_scroll_bar = tk.Scrollbar(self.canvas_frame, orient=tk.VERTICAL, command=self.primary_canvas.yview,
                                         bg=PrimaryStyle.PRIMARY_COLOR, width=11)
        self.v_scroll_bar.grid(row=0, column=1, sticky=tk.NSEW)
        self.v_scroll_bar.lift(self.scroll_window)

        self.primary_canvas.config(yscrollcommand=self.v_scroll_bar.set,
                                   scrollregion=self.primary_canvas.bbox("all"))

        self.del_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.del_frame.grid(row=2, column=0, sticky=tk.NSEW, columnspan=2)
        self.del_frame.grid_rowconfigure(index=0, weight=1)
        self.del_frame.grid_columnconfigure(index=0, weight=1)

        self.del_button = tk.Button(self.del_frame, text='Delete', command=lambda: self._delete_selection(),
                                    background=PrimaryStyle.PRIMARY_COLOR,
                                    fg=PrimaryStyle.FONT_COLOR)
        self.del_button.grid(row=0, column=0, sticky=tk.NSEW, )

        self.scroll_window.bind('<Configure>', self._configure_window)
        self.scroll_window.bind('<Enter>', self._bound_to_mousewheel)
        self.scroll_window.bind('<Leave>', self._unbound_to_mousewheel)

    def _add_selection(self):
        index = len(self.items)
        self.items.append(ToggleButton(self.scroll_window, top_level_scroll=self, index=index,
                                       width=72, height=72, text='TEST:%s' % index,
                                       bg=PrimaryStyle.SECONDARY_COLOR,
                                       fg=PrimaryStyle.FONT_COLOR))
        self.items[-1].pack(side=tk.TOP, fill=None, anchor=tk.CENTER)

        self._handle_prev_selected()

        self.curr_select = len(self.items) - 1
        self.items[self.curr_select].button.config(background=PrimaryStyle.SELECT_COLOR)
        self.items[self.curr_select].button.update()
        self.root.set_visibility(visible=True)

        self.root.add_item()
        name = self.root.update_gui(index=self.curr_select)
        if name is not None:
            self.items[self.curr_select].button.configure(text=name)
            self.items[self.curr_select].button.update()

    def _delete_selection(self):
        if self.curr_select is not None:
            tmp = self.items[self.curr_select]
            self.items.remove(tmp)
            tmp.destroy()
            del tmp
            self.root.delete_item(index=self.curr_select)
            self.curr_select = None
            self.root.set_visibility(visible=False)
            self.scroll_window.update()

        self._recalc_indexes()
        self.root.update_gui(index=self.curr_select)

    def _recalc_indexes(self):
        for ind in range(len(self.items)):
            self.items[ind].index = ind

    def select(self, index):
        self._handle_prev_selected()
        self.curr_select = index
        self.items[self.curr_select].button.configure(bg=PrimaryStyle.SELECT_COLOR)
        self.items[self.curr_select].button.update()
        self.root.set_visibility(visible=True)
        print('selected button at index: %s' % index)
        name = self.root.update_gui(index=self.curr_select)
        if name is not None:
            self.items[self.curr_select].button.configure(text=name)
            self.items[self.curr_select].button.update()

    def update_curr_name(self, name):
        self.items[self.curr_select].button.configure(text=name)
        self.items[self.curr_select].button.update()

    def update_from_list(self, names):
        for index, name in enumerate(names):
            self.items.append(ToggleButton(self.scroll_window, top_level_scroll=self, index=index,
                                           width=72, height=72, text=name,
                                           bg=PrimaryStyle.SECONDARY_COLOR,
                                           fg=PrimaryStyle.FONT_COLOR))
            self.items[-1].pack(side=tk.TOP, fill=None, anchor=tk.CENTER)

        self.curr_select = len(self.items) - 1
        self.items[self.curr_select].button.config(background=PrimaryStyle.SELECT_COLOR)
        self.items[self.curr_select].button.update()
        self.root.set_visibility(visible=True)
        self.root.update_gui(index=self.curr_select)

    def reset(self):
        for item in reversed(self.items):
            tmp = item
            self.items.remove(tmp)
            tmp.destroy()
            del tmp
        self.items = list()
        self.curr_select = None

    def _handle_prev_selected(self):
        if self.curr_select is not None:
            self.items[self.curr_select].button.configure(bg=PrimaryStyle.SECONDARY_COLOR)
            self.items[self.curr_select].button.update()
            name = self.root.get_curr_name()
            self.root.save_wing_settings()
            if name is not None:
                self.items[self.curr_select].button.configure(text=name)
                self.items[self.curr_select].button.update()

    def _bound_to_mousewheel(self, event):
        self.primary_canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _unbound_to_mousewheel(self, event):
        self.primary_canvas.unbind_all("<MouseWheel>")

    def _on_mousewheel(self, event):
        self.primary_canvas.yview_scroll(int(-1 * (event.delta / 120.0)), "units")

    def _configure_window(self, event):
        # update the scrollbars to match the size of the inner frame
        size = (self.scroll_window.winfo_reqwidth(), self.scroll_window.winfo_reqheight())
        self.primary_canvas.config(scrollregion='0 0 %s %s' % size)
        if self.scroll_window.winfo_reqwidth() != self.primary_canvas.winfo_width():
            # update the canvas's width to fit the inner frame
            self.primary_canvas.config(width=self.scroll_window.winfo_reqwidth())
        if self.scroll_window.winfo_reqheight() != self.primary_canvas.winfo_height():
            # update the canvas's width to fit the inner frame
            self.primary_canvas.config(height=self.scroll_window.winfo_reqheight())


class PlotWindow(tk.Frame):
    def __init__(self, master, root, **kwargs):
        super(PlotWindow, self).__init__(master, **kwargs)
        self.root = root

        self.pack_propagate(False)

        self.draw_figure = Figure(figsize=(4, 4), dpi=100)
        self.draw_figure.set_facecolor(PrimaryStyle.SECONDARY_COLOR)
        self.draw_canvas = FigureCanvasTkAgg(self.draw_figure, master=self)
        self.draw_canvas.get_tk_widget().pack(expand=True)

    def plot(self, callback):
        self.draw_figure.clear()
        self.draw_figure.clf()

        plot_1 = self.draw_figure.add_subplot(111)
        plot_1.set_facecolor(PrimaryStyle.SECONDARY_COLOR)
        plot_1.spines['bottom'].set_color(PrimaryStyle.FONT_COLOR)
        plot_1.spines['top'].set_color(PrimaryStyle.FONT_COLOR)
        plot_1.spines['left'].set_color(PrimaryStyle.FONT_COLOR)
        plot_1.spines['right'].set_color(PrimaryStyle.FONT_COLOR)
        for label in plot_1.xaxis.get_ticklabels():
            label.set_color(PrimaryStyle.FONT_COLOR)
        for label in plot_1.yaxis.get_ticklabels():
            label.set_color(PrimaryStyle.FONT_COLOR)
        for line in plot_1.yaxis.get_ticklines():
            line.set_color(PrimaryStyle.FONT_COLOR)
        for line in plot_1.xaxis.get_ticklines():
            line.set_color(PrimaryStyle.FONT_COLOR)
        for line in plot_1.xaxis.get_gridlines():
            line.set_color(PrimaryStyle.FONT_COLOR)

        for line in plot_1.yaxis.get_gridlines():
            line.set_color(PrimaryStyle.FONT_COLOR)
            line.set_markeredgewidth(8)

        # Call the plotting callback for the given object
        callback(plot_1)

        self.draw_canvas.draw()


class MainWindow(tk.Tk):
    def __init__(self, title, width, height):
        super(MainWindow, self).__init__(baseName=title)
        self.width = width
        self.height = height
        self.window_title = title
        self.title(title)

        self.curr_project = pm.Project.default_project()

        self.geometry("%sx%s" % (self.width, self.height))

        self.active_window = WindowState.HOME

        self.primary_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR)
        # self.primary_frame.grid_configure(column=1, row=1)
        self.primary_frame.pack(side='top', fill='both', expand=True)
        self.primary_frame.grid_columnconfigure(0, weight=1)
        self.primary_frame.grid_rowconfigure(0, weight=1)

        self.embedded_windows = [
            HomeWindow(self.primary_frame, self),
            MachineWindow(self.primary_frame, self),
            WingWindow(self.primary_frame, self),
            CADWindow(self.primary_frame, self),
            DatabaseWindow(self.primary_frame, self)
        ]

        for frame in self.embedded_windows:
            frame.grid(row=0, column=0, sticky=tk.NSEW)

        self.menu_bar = tk.Menu(self)
        self._create_menu_bar()

        self.switch_embedded_window(WindowState.HOME)

        self.resizable(False, False)

    def get_window_instance(self, window):
        return self.embedded_windows[window]

    def switch_embedded_window(self, window_enum):
        self.embedded_windows[window_enum].tkraise()
        print('Changing Window')
        self.update()

    def new_project(self):
        self.update()
        output_file = filedialog.asksaveasfilename(title="Create Project", master=self, defaultextension='.proj',
                                                   filetypes=[('Project', '*.proj')])
        self.curr_project = pm.Project(os.path.dirname(output_file), os.path.splitext(os.path.basename(output_file))[0])
        self.title(self.window_title + ' Project: %s' % self.curr_project.name)
        self.embedded_windows[WindowState.WING].reset()
        self.embedded_windows[WindowState.WING].scroll_frame.reset()
        self.save_project()

    def save_project(self):
        self.embedded_windows[WindowState.WING].save_wing_settings()
        if self.curr_project.name in 'Default':
            output_file = filedialog.asksaveasfilename(title="Create Project", master=self, defaultextension='.proj',
                                                       filetypes=[('Project', '*.proj')])
            self.curr_project = pm.Project(os.path.dirname(output_file),
                                           os.path.splitext(os.path.basename(output_file))[0])
            self.title(self.window_title + ' Project: %s' % self.curr_project.name)
        self.curr_project.wings = self.embedded_windows[WindowState.WING].wings
        self.curr_project.machines = self.embedded_windows[WindowState.MACHINE_SETUP].machines
        self.curr_project.cad_parts = self.embedded_windows[WindowState.CAD].cad_parts
        self.curr_project.save_project()

    def load_project(self):
        self.update()
        file_path = filedialog.askopenfilename(title="Open Project", master=self,
                                               filetypes=[('Project', '*.proj')])
        self.curr_project = pm.Project.load_project(file_path)
        self.title(self.window_title + ' Project: %s' % self.curr_project.name)
        self.embedded_windows[WindowState.WING].update_from_project()
        self.embedded_windows[WindowState.WING].wings = copy.deepcopy(self.curr_project.wings)
        self.embedded_windows[WindowState.MACHINE_SETUP].machines = copy.deepcopy(self.curr_project.machines)
        self.embedded_windows[WindowState.CAD].cad_parts = copy.deepcopy(self.curr_project.cad_parts)

    def exit(self):
        self.destroy()

    def _create_menu_bar(self):
        file_menu = tk.Menu(self.menu_bar, tearoff=0)
        file_menu.add_command(label="New Project", command=self.new_project)
        file_menu.add_command(label="Save Project", command=self.save_project)
        file_menu.add_command(label="Load Project", command=self.load_project)
        file_menu.add_command(label="Exit", command=self.exit)
        self.menu_bar.add_cascade(label="File", menu=file_menu)

        project_menu = tk.Menu(self.menu_bar, tearoff=0)
        project_menu.add_command(label="Home", command=lambda: self.switch_embedded_window(WindowState.HOME))
        project_menu.add_command(label="CNC Wire Cutter",
                                 command=lambda: self.switch_embedded_window(WindowState.MACHINE_SETUP))
        project_menu.add_command(label="Wing Design", command=lambda: self.switch_embedded_window(WindowState.WING))
        project_menu.add_command(label="CAD Import", command=lambda: self.switch_embedded_window(WindowState.CAD))
        project_menu.add_command(label="Edit Database",
                                 command=lambda: self.switch_embedded_window(WindowState.DATABASE))
        self.menu_bar.add_cascade(label="Project", menu=project_menu)

        help_menu = tk.Menu(self.menu_bar, tearoff=0)
        help_menu.add_command(label="About")
        help_menu.add_command(label="Users Guide")
        help_menu.add_command(label="Update")
        self.menu_bar.add_cascade(label="Help", menu=help_menu)

        self.config(menu=self.menu_bar)


class EmbeddedWindow(tk.Frame):
    def __init__(self, master, window_type, root):
        super(EmbeddedWindow, self).__init__(master, background=PrimaryStyle.PRIMARY_COLOR)
        self.window_type = window_type
        self.root = root

    def update_from_project(self):
        raise NotImplementedError('update_from_project not Implemented for WindowType: %s' % self.window_type)

    def add_item(self):
        raise NotImplementedError('add_item not Implemented for WindowType: %s' % self.window_type)

    def delete_item(self, index):
        raise NotImplementedError('delete_item not Implemented for WindowType: %s' % self.window_type)

    def update_gui(self, index):
        raise NotImplementedError('update_gui not Implemented for WindowType: %s' % self.window_type)

    def get_curr_name(self):
        raise NotImplementedError('get_curr_name not Implemented for WindowType: %s' % self.window_type)

    def reset(self):
        raise NotImplementedError('reset not Implemented for WindowType: %s' % self.window_type)


class HomeWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(HomeWindow, self).__init__(master, WindowState.HOME, root)

        self.grid_columnconfigure(index=0, weight=5)
        self.grid_columnconfigure(index=1, weight=1)
        self.grid_rowconfigure(index=0, weight=1)
        self.grid_propagate(False)

        self.left_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR)
        self.left_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=(PrimaryStyle.MAIN_FRAME_PAD, 0),
                             pady=PrimaryStyle.MAIN_FRAME_PAD)
        self.left_frame.grid_propagate(False)
        self.left_frame.grid_rowconfigure(index=0, weight=1)
        self.left_frame.grid_rowconfigure(index=1, weight=9)
        self.right_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR)
        self.right_frame.grid(row=0, column=1, sticky=tk.NSEW, padx=(0, PrimaryStyle.MAIN_FRAME_PAD),
                              pady=PrimaryStyle.MAIN_FRAME_PAD)
        self.right_frame.grid_propagate(False)

        self.left_top_frame = tk.Frame(self.left_frame, background=PrimaryStyle.PRIMARY_COLOR)
        self.left_top_frame.grid(row=0, column=0, sticky=tk.NSEW)
        self.left_bot_frame = tk.Frame(self.left_frame, background=PrimaryStyle.PRIMARY_COLOR)
        self.left_bot_frame.grid(row=1, column=0, sticky=tk.NSEW, )
        self.left_bot_frame.grid_propagate(False)

        btn_size = PrimaryStyle.HOME_BTN_SIZE
        self.mach_frame = tk.Frame(self.left_top_frame, width=btn_size, height=btn_size)
        self.wing_frame = tk.Frame(self.left_top_frame, width=btn_size, height=btn_size)
        self.body_frame = tk.Frame(self.left_top_frame, width=btn_size, height=btn_size)
        self.db_frame = tk.Frame(self.left_top_frame, width=btn_size, height=btn_size)

        self.machine_button = tk.Button(self.mach_frame, text="CNC Wire \nCutter",
                                        command=lambda: self.root.switch_embedded_window(WindowState.MACHINE_SETUP),
                                        bg=PrimaryStyle.SECONDARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.wing_button = tk.Button(self.wing_frame, text="Wing \nDesigner",
                                     command=lambda: self.root.switch_embedded_window(WindowState.WING),
                                     bg=PrimaryStyle.SECONDARY_COLOR,
                                     fg=PrimaryStyle.FONT_COLOR)
        self.body_button = tk.Button(self.body_frame, text="CAD \nImport",
                                     command=lambda: self.root.switch_embedded_window(WindowState.CAD),
                                     bg=PrimaryStyle.SECONDARY_COLOR,
                                     fg=PrimaryStyle.FONT_COLOR)
        self.database_button = tk.Button(self.db_frame, text="Edit \nDatabase",
                                         command=lambda: self.root.switch_embedded_window(WindowState.DATABASE),
                                         bg=PrimaryStyle.SECONDARY_COLOR,
                                         fg=PrimaryStyle.FONT_COLOR)

        for ind, frame in enumerate([self.mach_frame, self.wing_frame, self.body_frame, self.db_frame]):
            frame.grid_propagate(False)
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)
            if ind == 0:
                frame.grid(row=0, column=ind, padx=(0, PrimaryStyle.HOME_BTN_PADX),
                           pady=(PrimaryStyle.HOME_BTN_PADY, 0))
            elif ind == 3:
                frame.grid(row=0, column=ind, padx=(PrimaryStyle.HOME_BTN_PADX, 0),
                           pady=(PrimaryStyle.HOME_BTN_PADY, 0))
            else:
                frame.grid(row=0, column=ind, padx=PrimaryStyle.HOME_BTN_PADX, pady=(PrimaryStyle.HOME_BTN_PADY, 0))

        for btn in [self.machine_button, self.body_button, self.wing_button, self.database_button]:
            btn.grid(sticky=tk.NSEW)

        self.news_label = tk.Label(self.left_bot_frame, text='News:', bg=PrimaryStyle.TETRARY_COLOR,
                                   relief=tk.SOLID, borderwidth=1, justify=tk.LEFT,
                                   fg=PrimaryStyle.FONT_COLOR)
        self.news_label.pack(fill=tk.X, anchor=tk.NW, expand=False)

        self.news_box = tk.Text(self.left_bot_frame, background=PrimaryStyle.SECONDARY_COLOR, width=0,
                                fg=PrimaryStyle.FONT_COLOR)
        self.news_box.insert('end', 'Version 0.01: Gui is being added, this may or may not work, Yakka Yakka Yakka')
        self.news_box.config(wrap=tk.WORD)
        self.news_box.pack(fill=tk.BOTH, anchor=tk.NW, expand=True)

        self.proj_list = list()
        self.proj_ref = dict()
        self.proj_var = tk.StringVar(value=self.proj_list)

        self.proj_label = tk.Label(self.right_frame, text='Recent Projects:', bg=PrimaryStyle.TETRARY_COLOR,
                                   fg=PrimaryStyle.FONT_COLOR,
                                   borderwidth=1, relief=tk.SOLID)
        self.proj_label.pack(fill=tk.X, anchor=tk.NW, expand=False)
        self.recent_projects = tk.Listbox(self.right_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                          listvariable=self.proj_var,
                                          fg=PrimaryStyle.FONT_COLOR)
        self.recent_projects.pack(fill=tk.BOTH, anchor=tk.CENTER, expand=True, padx=0, pady=0)

    def fill_recent_projects(self, project_reference):
        """
        Fills the contents of the recent projects list box.

        :param project_reference: list of tuples defining project names and references.
        """
        proj_list = list()
        proj_ref = dict()

        for proj_name_tmp, proj_ref_tmp in project_reference:
            proj_list.append(proj_name_tmp)
            proj_ref[proj_name_tmp] = proj_ref_tmp

        self.proj_list = proj_list
        self.proj_ref = proj_ref
        self.proj_var.set(self.proj_list)


class MachineWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(MachineWindow, self).__init__(master, WindowState.MACHINE_SETUP, root)

        self.machines = list()
        self.machines.append(wc.WireCutter(name='short', wire_length=245, max_height=300.0, max_speed=150.0,
                                           min_speed=30, release_height=80.0, start_height=10.0, start_depth=20.0,
                                           dynamic_tension=True))
        self.machines[-1].set_dynamic_tension_spool_radius(7.79)
        self.machines[-1].set_dynamic_tension_motor_letter('V')
        self.machines[-1].reverse_dynamic_tension(True)

        self.machines.append(wc.WireCutter(name='baseline', wire_length=640, max_height=300.0, max_speed=150.0,
                                           min_speed=50, release_height=80.0, start_height=10.0, start_depth=20.0,
                                           dynamic_tension=True))
        self.machines[-1].set_dynamic_tension_spool_radius(7.79)
        self.machines[-1].set_dynamic_tension_motor_letter('V')
        self.machines[-1].reverse_dynamic_tension(True)

        self.machines.append(wc.WireCutter(name='extended', wire_length=1000, max_height=300.0, max_speed=150.0,
                                           min_speed=30, release_height=80.0, start_height=10.0, start_depth=20.0,
                                           dynamic_tension=True))
        self.machines[-1].set_dynamic_tension_spool_radius(7.79)
        self.machines[-1].set_dynamic_tension_motor_letter('V')
        self.machines[-1].reverse_dynamic_tension(True)

        self.grid_rowconfigure(index=0, weight=100)
        self.grid_columnconfigure(index=0, weight=1)
        self.grid_columnconfigure(index=1, weight=8)
        self.grid_propagate(False)

        self.primary_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.primary_frame.grid(row=0, column=1, sticky=tk.NSEW)
        self.primary_frame.grid_columnconfigure(index=0, weight=1)
        self.primary_frame.grid_columnconfigure(index=1, weight=3)
        self.primary_frame.grid_rowconfigure(index=0, weight=1)
        self.primary_frame.grid_propagate(False)
        self.primary_frame.pack_propagate(False)

        self.primary_frame.grid_rowconfigure(index=0, weight=1)
        self.primary_frame.grid_columnconfigure(index=0, weight=1)

        self.set_visibility(False)

        self.scroll_frame = ScrollableSelectionMenu(self, self)
        self.scroll_frame.grid(row=0, column=0, sticky=tk.NSEW)

        self._create_left_frame()

        self._create_right_frame()

    def set_visibility(self, visible):
        if visible:
            self.primary_frame.grid()
        else:
            self.primary_frame.grid_remove()

    def get_machine(self, tag):
        ret_val = None
        for machine in self.machines:
            if machine.name == tag:
                ret_val = machine
        return ret_val

    def _create_left_frame(self):
        self.left_frame = tk.Frame(self.primary_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)

        self.left_frame.grid(row=0, column=0, sticky=tk.NSEW)
        self.left_frame.grid_rowconfigure(index=0, weight=2)
        self.left_frame.grid_rowconfigure(index=1, weight=2)
        self.left_frame.grid_columnconfigure(index=0, weight=1)

        self.top_left_frame = tk.Frame(self.left_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.top_left_frame.grid(row=0, column=0, sticky=tk.NSEW)
        self.top_left_frame.grid_rowconfigure(index=0, weight=1)
        self.top_left_frame.grid_columnconfigure(index=0, weight=14)
        self.top_left_frame.grid_columnconfigure(index=1, weight=1)

        # ==============================================================================================================
        # Left Top Frame
        self.left_top_frame = tk.Frame(self.top_left_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.left_top_frame.grid(row=0, column=0, sticky=tk.NSEW)
        self.left_top_frame.grid_rowconfigure(index=0, weight=1)
        self.left_top_frame.grid_rowconfigure(index=1, weight=1)
        self.left_top_frame.grid_rowconfigure(index=2, weight=1)
        self.left_top_frame.grid_rowconfigure(index=3, weight=1)
        self.left_top_frame.grid_rowconfigure(index=4, weight=1)
        self.left_top_frame.grid_rowconfigure(index=5, weight=1)
        self.left_top_frame.grid_columnconfigure(index=0, weight=1)

        self.name_frame = tk.LabelFrame(self.left_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                        fg=PrimaryStyle.FONT_COLOR, text='Name:')
        self.name_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING,
                             pady=(PrimaryStyle.GENERAL_PADDING / 2, PrimaryStyle.GENERAL_PADDING / 4))
        self.name_frame.pack_propagate(False)

        self.name_text = tk.Text(self.name_frame)
        self.name_text.pack(pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 2),
                            padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.wire_frame = tk.LabelFrame(self.left_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                        fg=PrimaryStyle.FONT_COLOR, text='Wire Length (mm):')
        self.wire_frame.grid(row=1, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING,
                             pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 4))
        self.wire_frame.pack_propagate(False)

        self.wire_text = tk.Text(self.wire_frame)
        self.wire_text.pack(pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 2),
                            padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.height_frame = tk.LabelFrame(self.left_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                          highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                          highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                          fg=PrimaryStyle.FONT_COLOR, text='Max Height (mm):')
        self.height_frame.grid(row=2, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING,
                               pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 4))
        self.height_frame.pack_propagate(False)

        self.height_text = tk.Text(self.height_frame)
        self.height_text.pack(pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 2),
                              padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.depth_frame = tk.LabelFrame(self.left_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                         highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                         highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                         fg=PrimaryStyle.FONT_COLOR, text='Max Depth (mm):')
        self.depth_frame.grid(row=3, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING,
                              pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 4))
        self.depth_frame.pack_propagate(False)

        self.depth_text = tk.Text(self.depth_frame)
        self.depth_text.pack(pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 2),
                             padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.fast_frame = tk.LabelFrame(self.left_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                        fg=PrimaryStyle.FONT_COLOR, text='Fast Speed (mm/min):')
        self.fast_frame.grid(row=4, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING,
                             pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 4))
        self.fast_frame.pack_propagate(False)

        self.fast_text = tk.Text(self.fast_frame)
        self.fast_text.pack(pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 2),
                            padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.slow_frame = tk.LabelFrame(self.left_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                        fg=PrimaryStyle.FONT_COLOR, text='Slow Speed (mm/min):')
        self.slow_frame.grid(row=5, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING,
                             pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 2))
        self.slow_frame.pack_propagate(False)

        self.slow_text = tk.Text(self.slow_frame)
        self.slow_text.pack(pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 2),
                            padx=PrimaryStyle.GENERAL_PADDING / 2)

        # ==============================================================================================================
        # Right Top Frame
        self.right_top_frame = tk.Frame(self.top_left_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.right_top_frame.grid(row=0, column=1, sticky=tk.NSEW)
        self.right_top_frame.grid_rowconfigure(index=0, weight=2)
        self.right_top_frame.grid_rowconfigure(index=1, weight=1)
        self.right_top_frame.grid_rowconfigure(index=2, weight=2)
        self.right_top_frame.grid_columnconfigure(index=0, weight=1)

        self.axis_def_frame = tk.LabelFrame(self.right_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR,
                                            text='Axis Definitions')
        self.axis_def_frame.grid(row=0, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 2,
                                 padx=(0, PrimaryStyle.GENERAL_PADDING / 2))
        self.axis_def_frame.grid_rowconfigure(index=0, weight=1)
        self.axis_def_frame.grid_rowconfigure(index=1, weight=1)
        self.axis_def_frame.grid_columnconfigure(index=0, weight=1)
        self.axis_def_frame.grid_columnconfigure(index=1, weight=1)

        self.x1_frame = tk.LabelFrame(self.axis_def_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                      fg=PrimaryStyle.FONT_COLOR,
                                      text='X1:')
        self.x1_frame.grid(row=0, column=0, sticky=tk.NSEW, pady=(2, 1),
                           padx=(2, 1))
        self.x1_frame.pack_propagate(False)
        self.x1_text = tk.Text(self.x1_frame)
        self.x1_text.insert('end', 'X')
        self.x1_text.pack(pady=PrimaryStyle.GENERAL_PADDING / 2, padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.y1_frame = tk.LabelFrame(self.axis_def_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                      fg=PrimaryStyle.FONT_COLOR,
                                      text='Y1:')
        self.y1_frame.grid(row=0, column=1, sticky=tk.NSEW, pady=(2, 1),
                           padx=(1, 2))
        self.y1_frame.pack_propagate(False)
        self.y1_text = tk.Text(self.y1_frame)
        self.y1_text.insert('end', 'Y')
        self.y1_text.pack(pady=PrimaryStyle.GENERAL_PADDING / 2, padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.x2_frame = tk.LabelFrame(self.axis_def_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                      fg=PrimaryStyle.FONT_COLOR,
                                      text='X2:')
        self.x2_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=(1, 2),
                           padx=(2, 1))
        self.x2_frame.pack_propagate(False)
        self.x2_text = tk.Text(self.x2_frame)
        self.x2_text.insert('end', 'U')
        self.x2_text.pack(pady=PrimaryStyle.GENERAL_PADDING / 2, padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.y2_frame = tk.LabelFrame(self.axis_def_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                      fg=PrimaryStyle.FONT_COLOR,
                                      text='Y2:')
        self.y2_frame.grid(row=1, column=1, sticky=tk.NSEW, pady=(1, 2),
                           padx=(1, 2))
        self.y2_frame.pack_propagate(False)
        self.y2_text = tk.Text(self.y2_frame)
        self.y2_text.insert('end', 'Z')
        self.y2_text.pack(pady=PrimaryStyle.GENERAL_PADDING / 2, padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.feed_frame = tk.LabelFrame(self.right_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                        fg=PrimaryStyle.FONT_COLOR,
                                        text='Feed Rate Mode')
        self.feed_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 2,
                             padx=(0, PrimaryStyle.GENERAL_PADDING / 2))
        self.feed_frame.grid_rowconfigure(index=0, weight=1)
        self.feed_frame.grid_rowconfigure(index=1, weight=1)
        self.feed_frame.grid_rowconfigure(index=2, weight=1)
        self.feed_frame.grid_rowconfigure(index=3, weight=1)
        self.feed_frame.grid_rowconfigure(index=4, weight=1)
        self.feed_frame.grid_columnconfigure(index=1, weight=1)
        self.feed_frame.grid_propagate(False)

        self.inv_time_check = tk.Checkbutton(self.feed_frame, text='Inverse Time',
                                             background=PrimaryStyle.SECONDARY_COLOR,
                                             highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                             highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                             fg=PrimaryStyle.FONT_COLOR)
        self.inv_time_check.grid(row=0, column=0, sticky=tk.NW)

        self.upt_check = tk.Checkbutton(self.feed_frame, text='Units/Min',
                                        background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.upt_check.grid(row=1, column=0, sticky=tk.NW)

        self.option_frame = tk.LabelFrame(self.right_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                          highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                          highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                          fg=PrimaryStyle.FONT_COLOR,
                                          text='Options')
        self.option_frame.grid(row=2, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 2,
                               padx=(0, PrimaryStyle.GENERAL_PADDING / 2))
        self.option_frame.grid_rowconfigure(index=0, weight=1)
        self.option_frame.grid_rowconfigure(index=1, weight=1)
        self.option_frame.grid_columnconfigure(index=1, weight=1)
        self.option_frame.grid_propagate(False)

        self.default_check = tk.Checkbutton(self.option_frame, text='Default',
                                            background=PrimaryStyle.SECONDARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR)
        self.default_check.grid(row=0, column=0, sticky=tk.NW)

        # ==============================================================================================================
        # Bottom Frame
        self.bot_left_frame = tk.Frame(self.left_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.bot_left_frame.grid(row=1, column=0, sticky=tk.NSEW)
        self.bot_left_frame.grid_rowconfigure(index=0, weight=1)
        self.bot_left_frame.grid_columnconfigure(index=0, weight=1)

        self.start_up_gcode = tk.LabelFrame(self.bot_left_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR,
                                            text='Machine Startup G-Code')
        self.start_up_gcode.grid(row=0, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING,
                                 padx=(PrimaryStyle.GENERAL_PADDING, PrimaryStyle.GENERAL_PADDING / 2))
        self.start_up_gcode.pack_propagate(False)

        self.gcode_text_field = tk.Text(self.start_up_gcode, background=PrimaryStyle.SECONDARY_COLOR, width=0,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.gcode_text_field.config(wrap=tk.WORD)

        self.gcode_text_field.insert('end', 'G17\nG21')
        self.gcode_text_field.pack(fill=tk.BOTH, anchor=tk.NW, expand=True, pady=PrimaryStyle.GENERAL_PADDING,
                                   padx=PrimaryStyle.GENERAL_PADDING / 2)

    def _create_right_frame(self):
        self.right_frame = tk.Frame(self.primary_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                    highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                    highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)

        self.right_frame.grid(row=0, column=1, sticky=tk.NSEW)
        self.right_frame.grid_columnconfigure(index=0, weight=1)
        self.right_frame.grid_rowconfigure(index=0, weight=1)
        self.right_frame.pack_propagate(False)
        self.right_frame.grid_propagate(False)

        self.gantry_photo = tk.PhotoImage(file=os.path.join(r'assets\gui\gantries.png'), width=900, height=900)
        self.gantry_photo = self.gantry_photo.subsample(2)
        self.photo_label = tk.Label(self.right_frame, image=self.gantry_photo)
        self.photo_label.grid(row=0, column=0, sticky=tk.NSEW)


class WingWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(WingWindow, self).__init__(master, WindowState.WING, root)
        self.main_window = root
        self.wings = list()
        self.curr_selected = None

        self.logger = logging.getLogger(__name__)

        self.grid_rowconfigure(index=0, weight=100)
        self.grid_columnconfigure(index=0, weight=1)
        self.grid_columnconfigure(index=1, weight=8)
        self.grid_propagate(False)

        self.primary_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.primary_frame.grid(row=0, column=1, sticky=tk.NSEW)

        self.primary_frame.grid_rowconfigure(index=0, weight=1)
        self.primary_frame.grid_columnconfigure(index=0, weight=1)
        self.primary_frame.grid_propagate(False)

        self.primary_frame.grid_remove()

        self.scroll_frame = ScrollableSelectionMenu(self, self)
        self.scroll_frame.grid(row=0, column=0, sticky=tk.NSEW)

        self._create_top_frame()

    def set_visibility(self, visible):
        if visible:
            self.primary_frame.grid()
        else:
            self.primary_frame.grid_remove()

    def _create_top_frame(self):
        self.top_frame = tk.Frame(self.primary_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.top_frame.grid(row=0, column=0, sticky=tk.NSEW)

        self.top_frame.grid_columnconfigure(index=0, weight=2)
        self.top_frame.grid_columnconfigure(index=1, weight=1)
        self.top_frame.grid_rowconfigure(index=0, weight=1)

        # ==============================================================================================================
        # LEFT TOP
        self.left_top_frame = tk.Frame(self.top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.left_top_frame.grid(row=0, column=0, sticky=tk.NSEW)
        self.left_top_frame.grid_rowconfigure(index=0, weight=1)
        self.left_top_frame.grid_rowconfigure(index=1, weight=1)
        self.left_top_frame.grid_columnconfigure(index=0, weight=1)

        self.top_airfoil_frame = self.AirfoilFrame(self.left_top_frame, self, position=0,
                                                   background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                   text='Section XY:', fg=PrimaryStyle.FONT_COLOR)
        self.top_airfoil_frame.grid(row=0, column=0, sticky=tk.NSEW, pady=(PrimaryStyle.GENERAL_PADDING / 2, 0),
                                    padx=(PrimaryStyle.GENERAL_PADDING / 2, 0))

        self.bot_airfoil_frame = self.AirfoilFrame(self.left_top_frame, self, position=1,
                                                   background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                   text='Section UZ:', fg=PrimaryStyle.FONT_COLOR)
        self.bot_airfoil_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=(0, PrimaryStyle.GENERAL_PADDING / 2),
                                    padx=(PrimaryStyle.GENERAL_PADDING / 2, 0))

        # ==============================================================================================================
        # RIGHT TOP

        self.right_top_frame = tk.Frame(self.top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.right_top_frame.grid(row=0, column=1, sticky=tk.NSEW)
        self.right_top_frame.grid_rowconfigure(index=0, weight=7)
        self.right_top_frame.grid_rowconfigure(index=1, weight=5)
        self.right_top_frame.grid_rowconfigure(index=2, weight=1)
        self.right_top_frame.grid_columnconfigure(index=0, weight=1)

        self.wing_setting_frame = WingWindow.WingSettingFrame(self.right_top_frame, root=self,
                                                              background=PrimaryStyle.SECONDARY_COLOR,
                                                              highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                              highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.wing_setting_frame.grid(row=0, column=0, sticky=tk.NSEW)

        # ==============================================================================================================
        # Spar Frame
        self.spar_frame = WingWindow.SparWindow(self.right_top_frame, root=self,
                                                background=PrimaryStyle.SECONDARY_COLOR,
                                                highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.spar_frame.grid(row=1, column=0, sticky=tk.NSEW)

        # ==========================================================================================================
        # Slicer buttons
        self.slice_btn_frame = tk.Frame(self.right_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.slice_btn_frame.grid(row=2, column=0, sticky=tk.NSEW)
        self.slice_btn_frame.grid_rowconfigure(index=0, weight=1)
        self.slice_btn_frame.grid_columnconfigure(index=0, weight=1)
        self.slice_btn_frame.grid_columnconfigure(index=1, weight=1)
        self.slice_btn_frame.grid_propagate(False)

        self.slice_btn = tk.Button(self.slice_btn_frame, text='Slice\nSelected', command=self.slice_selected)
        self.slice_btn.grid(row=0, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING / 2,
                            pady=PrimaryStyle.GENERAL_PADDING / 2)

        self.slice_all_btn = tk.Button(self.slice_btn_frame, text='Slice\nAll', command=self.slice_all)
        self.slice_all_btn.grid(row=0, column=1, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING / 2,
                                pady=PrimaryStyle.GENERAL_PADDING / 2)

    def _get_xlim_for_plotting(self):
        xmin = 0
        xmax = 1
        chord1 = self.top_airfoil_frame.chord_text_box.get(1.0, "end-1c")
        chord2 = self.bot_airfoil_frame.chord_text_box.get(1.0, "end-1c")

        chord1 = float(chord1) if is_float(chord1) else None
        chord2 = float(chord2) if is_float(chord2) else None

        span = self.wing_setting_frame.span_text.get(1.0, 'end-1c')
        span = float(span) if is_float(span) else None

        sweep = self.wing_setting_frame.sweep_text.get(1.0, 'end-1c')
        sweep = float(sweep) if is_float(sweep) else None

        if chord1 is not None and chord2 is not None:
            xmax = max(chord1, chord2)
        elif chord1 is not None:
            xmax = chord1
        elif chord2 is not None:
            xmax = chord2

        if span is not None and sweep is not None:
            offset = np.sin(np.deg2rad(sweep)) * span
            if offset < 0:
                xmin = offset
            elif chord1 is not None and chord2 is not None:
                xmax = max(chord1, chord2 + offset)
            else:
                xmin = 0

        return [xmin, xmax]

    def update_airfoil_plots(self, event):
        self.top_airfoil_frame.plot_airfoil(event)
        self.bot_airfoil_frame.plot_airfoil(event)

    class AirfoilFrame(tk.LabelFrame):
        def __init__(self, master, root, position, **kwargs):
            super(WingWindow.AirfoilFrame, self).__init__(master, **kwargs)
            self.root = root
            self.position = position

            self.grid_rowconfigure(index=0, weight=5)
            self.grid_rowconfigure(index=1, weight=1)
            self.grid_columnconfigure(index=0, weight=1)

            self.top_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.top_frame.grid(row=0, column=0, sticky=tk.NSEW)
            self.top_frame.grid_rowconfigure(index=0, weight=1)
            self.top_frame.grid_columnconfigure(index=0, weight=1)
            self.top_frame.grid_propagate(False)

            self.airfoil_frame = tk.Frame(self.top_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                          highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                          highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.airfoil_frame.grid(row=0, column=0, sticky=tk.NSEW)
            self.airfoil_frame.pack_propagate(False)

            self.draw_figure = Figure(figsize=(5, 2.2), dpi=100)
            self.draw_figure.set_facecolor(PrimaryStyle.SECONDARY_COLOR)
            self.draw_canvas = FigureCanvasTkAgg(self.draw_figure, master=self.airfoil_frame)
            self.draw_canvas.get_tk_widget().pack(expand=True)

            self.bot_frame = tk.Frame(self, background=PrimaryStyle.SECONDARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.bot_frame.grid(row=1, column=0, sticky=tk.NSEW)
            self.bot_frame.grid_rowconfigure(index=0, weight=1)
            self.bot_frame.grid_columnconfigure(index=0, weight=1)
            self.bot_frame.grid_columnconfigure(index=1, weight=1)
            self.bot_frame.grid_columnconfigure(index=2, weight=2)

            self.chord_label_frame = tk.LabelFrame(self.bot_frame, text='Chord', fg=PrimaryStyle.FONT_COLOR,
                                                   background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.chord_label_frame.grid(row=0, column=0, sticky=tk.NSEW,
                                        padx=PrimaryStyle.GENERAL_PADDING / 2,
                                        pady=PrimaryStyle.GENERAL_PADDING)
            self.chord_label_frame.grid_propagate(False)

            self.chord_text_box = tk.Text(self.chord_label_frame, height=1, width=8)
            self.chord_text_box.grid(row=0, column=0, sticky=tk.NSEW, padx=(4, 20), pady=1)
            self.chord_text_box.bind("<FocusOut>", self.root.update_airfoil_plots)
            self.chord_text_box.bind("<KeyRelease>", self.root.update_airfoil_plots)

            self.kerf_label_frame = tk.LabelFrame(self.bot_frame, text='Kerf', fg=PrimaryStyle.FONT_COLOR,
                                                  background=PrimaryStyle.SECONDARY_COLOR,
                                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.kerf_label_frame.grid(row=0, column=1, sticky=tk.NSEW,
                                       padx=PrimaryStyle.GENERAL_PADDING / 2,
                                       pady=PrimaryStyle.GENERAL_PADDING)
            self.kerf_label_frame.grid_propagate(False)

            self.kerf_text_box = tk.Text(self.kerf_label_frame, height=1, width=8)
            self.kerf_text_box.grid(row=0, column=0, sticky=tk.NSEW, padx=(4, 20), pady=1)

            self.airfoil_label_frame = tk.LabelFrame(self.bot_frame, text='Airfoil', fg=PrimaryStyle.FONT_COLOR,
                                                     background=PrimaryStyle.SECONDARY_COLOR,
                                                     highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                     highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.airfoil_label_frame.grid(row=0, column=2, sticky=tk.NSEW,
                                          padx=PrimaryStyle.GENERAL_PADDING / 2,
                                          pady=PrimaryStyle.GENERAL_PADDING)
            self.airfoil_label_frame.grid_propagate(False)

            self.selected_airfoil = tk.StringVar()
            self.selected_airfoil.set('Select Airfoil')

            self.airfoil_option_menu = ttk.Combobox(self.airfoil_label_frame, textvariable=self.selected_airfoil,
                                                    values=self.get_airfoil_options(), state='readonly')
            self.airfoil_option_menu.grid(row=0, column=0, sticky=tk.NSEW, padx=(4, 20), pady=1)

            self.airfoil_option_menu.bind('<<ComboboxSelected>>', self.plot_airfoil)

        def get_airfoil_options(self):
            airfoils = list()
            airfoils.extend(self.root.main_window.curr_project.database.airfoils.keys())
            return airfoils

        def plot_airfoil(self, event):
            print(event)
            airfoil_selection = self.airfoil_option_menu.get()
            self.draw_figure.clear()
            self.draw_figure.clf()
            if airfoil_selection not in ['Select Airfoil', '', ' ']:
                print('Plotting Airfoil')

                plot_1 = self.draw_figure.add_subplot(111)
                plot_1.set_facecolor(PrimaryStyle.SECONDARY_COLOR)
                plot_1.spines['bottom'].set_color(PrimaryStyle.FONT_COLOR)
                plot_1.spines['top'].set_color(PrimaryStyle.FONT_COLOR)
                plot_1.spines['left'].set_color(PrimaryStyle.FONT_COLOR)
                plot_1.spines['right'].set_color(PrimaryStyle.FONT_COLOR)
                for label in plot_1.xaxis.get_ticklabels():
                    label.set_color(PrimaryStyle.FONT_COLOR)
                for label in plot_1.yaxis.get_ticklabels():
                    label.set_color(PrimaryStyle.FONT_COLOR)
                for line in plot_1.yaxis.get_ticklines():
                    line.set_color(PrimaryStyle.FONT_COLOR)
                for line in plot_1.xaxis.get_ticklines():
                    line.set_color(PrimaryStyle.FONT_COLOR)
                for line in plot_1.xaxis.get_gridlines():
                    line.set_color(PrimaryStyle.FONT_COLOR)

                for line in plot_1.yaxis.get_gridlines():
                    line.set_color(PrimaryStyle.FONT_COLOR)
                    line.set_markeredgewidth(8)

                airfoil = gp.Dat(
                    data=self._transform_airfoil(self.root.main_window.curr_project.database.airfoils[airfoil_selection]))
                airfoil.plot_points_2d_gui(plot_1, PrimaryStyle.FONT_COLOR, PrimaryStyle.TETRARY_COLOR)

                xlim = self.root._get_xlim_for_plotting()
                plot_1.set_xlim(xlim[0], xlim[1])

            self.draw_canvas.draw()

        def _transform_airfoil(self, data):
            chord = self.chord_text_box.get(1.0, 'end-1c')
            chord = float(chord) if is_float(chord) else None

            washout = self.root.wing_setting_frame.washout_text.get(1.0, 'end-1c')
            washout = float(washout) if is_float(washout) else None

            sweep = self.root.wing_setting_frame.sweep_text.get(1.0, 'end-1c')
            sweep = float(sweep) if is_float(sweep) else None

            span = self.root.wing_setting_frame.span_text.get(1.0, 'end-1c')
            span = float(span) if is_float(span) else None

            points = copy.deepcopy(data)
            if chord is not None:
                spm.PointManip.Transform.scale(points, [chord, chord, 0])

            if washout is not None and self.position == 1:
                spm.PointManip.Transform.rotate(points, [0.0, 0.0, np.deg2rad(washout)])

            if sweep is not None and span is not None and self.position == 1:
                offset = np.sin(np.deg2rad(sweep)) * span
                spm.PointManip.Transform.translate(points, [offset, 0.0, 0.0])

            return points

    class WingSettingFrame(tk.Frame):
        def __init__(self, master, root, **kwargs):
            super(WingWindow.WingSettingFrame, self).__init__(master, **kwargs)
            self.root = root

            self.grid(row=0, column=1, sticky=tk.NSEW)
            self.grid_rowconfigure(index=0, weight=1)
            self.grid_columnconfigure(index=0, weight=1)

            self.wing_field_frame = tk.Frame(self, background=PrimaryStyle.SECONDARY_COLOR,
                                             highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                             highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.wing_field_frame.grid(row=0, column=0, sticky=tk.NSEW)
            self.wing_field_frame.grid_rowconfigure(index=0, weight=3)
            self.wing_field_frame.grid_rowconfigure(index=1, weight=3)
            self.wing_field_frame.grid_rowconfigure(index=2, weight=3)
            self.wing_field_frame.grid_rowconfigure(index=3, weight=3)
            self.wing_field_frame.grid_rowconfigure(index=4, weight=4)
            self.wing_field_frame.grid_rowconfigure(index=5, weight=3)
            self.wing_field_frame.grid_columnconfigure(index=0, weight=1)
            self.wing_field_frame.grid_propagate(False)

            self.name_label_frame = tk.LabelFrame(self.wing_field_frame, text='Name:',
                                                  background=PrimaryStyle.SECONDARY_COLOR,
                                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                  fg=PrimaryStyle.FONT_COLOR, width=5)
            self.name_label_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING / 2,
                                       pady=(PrimaryStyle.GENERAL_PADDING / 2, PrimaryStyle.GENERAL_PADDING / 4))
            self.name_label_frame.pack_propagate(False)

            self.name_text = tk.Text(self.name_label_frame)
            self.name_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 4),
                                padx=PrimaryStyle.GENERAL_PADDING / 2)
            self.name_text.bind("<KeyRelease>", self.update_name)
            self.name_text.bind("<FocusOut>", self.update_name)
            self.name_text.bind("<FocusIn>", self.update_name)

            self.span_label_frame = tk.LabelFrame(self.wing_field_frame, text='Span (mm):',
                                                  background=PrimaryStyle.SECONDARY_COLOR,
                                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                  fg=PrimaryStyle.FONT_COLOR, width=10)
            self.span_label_frame.grid(row=1, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING / 2,
                                       pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.span_label_frame.pack_propagate(False)

            self.span_text = tk.Text(self.span_label_frame)
            self.span_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 4,
                                padx=PrimaryStyle.GENERAL_PADDING / 2)
            self.span_text.bind("<KeyRelease>", self.root.update_airfoil_plots)
            self.span_text.bind("<FocusOut>", self.root.update_airfoil_plots)
            self.span_text.bind("<FocusIn>", self.root.update_airfoil_plots)

            self.washout_label_frame = tk.LabelFrame(self.wing_field_frame, text='Washout (deg):',
                                                     background=PrimaryStyle.SECONDARY_COLOR,
                                                     highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                     highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                     fg=PrimaryStyle.FONT_COLOR, width=14)
            self.washout_label_frame.grid(row=2, column=0, sticky=tk.NSEW,
                                          padx=PrimaryStyle.GENERAL_PADDING / 2,
                                          pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.washout_label_frame.pack_propagate(False)

            self.washout_text = tk.Text(self.washout_label_frame)
            self.washout_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 4,
                                   padx=PrimaryStyle.GENERAL_PADDING / 2)
            self.washout_text.bind("<KeyRelease>", self.root.update_airfoil_plots)
            self.washout_text.bind("<FocusOut>", self.root.update_airfoil_plots)
            self.washout_text.bind("<FocusIn>", self.root.update_airfoil_plots)

            self.sweep_label_frame = tk.LabelFrame(self.wing_field_frame, text='Sweep (deg):',
                                                   background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                   fg=PrimaryStyle.FONT_COLOR, width=14)
            self.sweep_label_frame.grid(row=3, column=0, sticky=tk.NSEW,
                                        padx=PrimaryStyle.GENERAL_PADDING / 2,
                                        pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.sweep_label_frame.pack_propagate(False)

            self.sweep_text = tk.Text(self.sweep_label_frame)
            self.sweep_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 4,
                                 padx=PrimaryStyle.GENERAL_PADDING / 2)
            self.sweep_text.bind("<KeyRelease>", self.root.update_airfoil_plots)
            self.sweep_text.bind("<FocusOut>", self.root.update_airfoil_plots)
            self.sweep_text.bind("<FocusIn>", self.root.update_airfoil_plots)

            self.options_frame = tk.LabelFrame(self.wing_field_frame,
                                               background=PrimaryStyle.SECONDARY_COLOR,
                                               highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                               highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                               width=20, text='Options:', fg=PrimaryStyle.FONT_COLOR)
            self.options_frame.grid(row=4, column=0, sticky=tk.NSEW,
                                    padx=PrimaryStyle.GENERAL_PADDING / 2,
                                    pady=PrimaryStyle.GENERAL_PADDING / 4)
            self.options_frame.grid_rowconfigure(index=0, weight=1)
            self.options_frame.grid_rowconfigure(index=1, weight=1)
            self.options_frame.grid_columnconfigure(index=0, weight=1)
            self.options_frame.grid_propagate(False)

            self.gen_lr_var = tk.IntVar(self)
            self.gen_left_right = tk.Checkbutton(self.options_frame, text='Create Left/Right', onvalue=1, offvalue=0,
                                                 variable=self.gen_lr_var, command=self.check_gen_lr,
                                                 background=PrimaryStyle.SECONDARY_COLOR,
                                                 selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                 fg=PrimaryStyle.FONT_COLOR)
            self.gen_left_right.grid(row=0, column=0, sticky=tk.W)

            self.align_led_var = tk.IntVar(self)
            self.align_led_edge = tk.Checkbutton(self.options_frame, text='Align to wire', variable=self.align_led_var,
                                                 onvalue=1, offvalue=0,
                                                 background=PrimaryStyle.SECONDARY_COLOR, command=self.check_align_led,
                                                 selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                 fg=PrimaryStyle.FONT_COLOR)
            self.align_led_edge.grid(row=1, column=0, sticky=tk.W)

            self.sel_cnc_machine_frame = tk.LabelFrame(self.wing_field_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                       fg=PrimaryStyle.FONT_COLOR,
                                                       text='Select CNC Machine:')
            self.sel_cnc_machine_frame.grid(row=5, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING / 2,
                                            pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 2))
            self.sel_cnc_machine_frame.grid_propagate(False)

            self.selected_machine = tk.StringVar()
            self.selected_machine.set('Select CNC Machine')

            self.cnc_machine_option_menu = ttk.Combobox(self.sel_cnc_machine_frame, textvariable=self.selected_machine,
                                                        values=[])
            self.cnc_machine_option_menu.grid(row=0, column=0, sticky=tk.NSEW, padx=(4, 20), pady=1)

        def check_align_led(self):
            # self.align_led_var.set(not self.align_led_var.get())
            print('check align led set to: %s' % self.align_led_var.get())
            if self.align_led_var.get() == 1:
                self.align_led_edge.configure(selectcolor=PrimaryStyle.TETRARY_COLOR)
            else:
                self.align_led_edge.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)

        def check_gen_lr(self):
            # self.gen_lr_var.set(not self.gen_lr_var.get())
            print('check gen lr set to: %s' % self.gen_lr_var.get())
            if self.gen_lr_var.get() == 1:
                self.gen_left_right.configure(selectcolor=PrimaryStyle.TETRARY_COLOR)
            else:
                self.gen_left_right.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)

        def get_machine_options(self):
            machines = list()
            for machine in self.root.root.get_window_instance(window=WindowState.MACHINE_SETUP).machines:
                machines.append(machine.name)
            return machines

        def update_name(self, event):
            self.root.scroll_frame.update_curr_name(name=self.name_text.get("1.0", "end-1c"))

        def _update_tip_airfoil(self, event):
            self.root.bot_airfoil_frame.plot_airfoil(event=0)

    class SparWindow(tk.Frame):
        def __init__(self, master, root, **kwargs):
            super(WingWindow.SparWindow, self).__init__(master, **kwargs)
            self.root = root

            self.grid_rowconfigure(index=0, weight=1)
            self.grid_columnconfigure(index=0, weight=1)

            self.spar_frame = tk.LabelFrame(self, text='Spars', background=PrimaryStyle.SECONDARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR)
            self.spar_frame.grid(row=0, column=0, sticky=tk.NSEW,
                                 padx=PrimaryStyle.GENERAL_PADDING / 2,
                                 pady=PrimaryStyle.GENERAL_PADDING / 2)

    def save_wing_settings(self):
        if self.curr_selected is not None:
            wing = self.wings[self.curr_selected]
            wing.name = self.wing_setting_frame.name_text.get("1.0", "end-1c")

            wing.machine_tag = self.wing_setting_frame.cnc_machine_option_menu.get()

            tmp_span = self.wing_setting_frame.span_text.get("1.0", "end-1c")
            tmp_washout = self.wing_setting_frame.washout_text.get("1.0", "end-1c")
            tmp_sweep = self.wing_setting_frame.sweep_text.get("1.0", "end-1c")

            tmp_root_chord = self.top_airfoil_frame.chord_text_box.get("1.0", "end-1c")
            tmp_tip_chord = self.bot_airfoil_frame.chord_text_box.get("1.0", "end-1c")

            tmp_root_kerf = self.top_airfoil_frame.kerf_text_box.get("1.0", "end-1c")
            tmp_tip_kerf = self.bot_airfoil_frame.kerf_text_box.get("1.0", "end-1c")

            try:
                wing.span = float(tmp_span)
                wing.washout = float(tmp_washout)
                wing.sweep = float(tmp_sweep)
                wing.tip_chord = float(tmp_tip_chord)
                wing.root_chord = float(tmp_root_chord)
                wing.tip_kerf = float(tmp_tip_kerf)
                wing.root_kerf = float(tmp_root_kerf)
            except ValueError:
                self.logger.info('Warning: One of the wing settings is not convertible to a number')

            wing.tip_airfoil_tag = self.bot_airfoil_frame.airfoil_option_menu.get()
            wing.root_airfoil_tag = self.top_airfoil_frame.airfoil_option_menu.get()
            if wing.tip_airfoil_tag != 'Select Airfoil':
                wing.tip_airfoil = copy.deepcopy(
                    self.root.curr_project.database.airfoils[wing.tip_airfoil_tag])
            if wing.root_airfoil_tag != 'Select Airfoil':
                wing.root_airfoil = copy.deepcopy(
                    self.root.curr_project.database.airfoils[wing.root_airfoil_tag])

            wing.symmetric = True if self.wing_setting_frame.gen_lr_var.get() == 1 else False
            wing.align_with_le = True if self.wing_setting_frame.align_led_var.get() == 1 else False

    def add_item(self):
        self.wings.append(cm.WingSegment(name='', logger=self.logger))
        # self.curr_selected = len(self.wings)-1

    def delete_item(self, index):
        tmp = self.wings[index]
        self.wings.remove(tmp)
        del tmp

    def update_gui(self, index):
        # Index is only none if we are removing an entry, do not save wing settings if we are deleting
        ret_val = None
        if index is None:
            self.set_visibility(False)
        else:
            self.save_wing_settings()
            self.curr_selected = index
            self.logger.info('gui wing index: %s', index)
            wing = self.wings[self.curr_selected]

            # Clear the text fields
            self.wing_setting_frame.name_text.delete(1.0, "end")
            self.wing_setting_frame.span_text.delete(1.0, "end")
            self.wing_setting_frame.washout_text.delete(1.0, "end")
            self.wing_setting_frame.sweep_text.delete(1.0, "end")
            self.top_airfoil_frame.chord_text_box.delete(1.0, "end")
            self.bot_airfoil_frame.chord_text_box.delete(1.0, "end")
            self.top_airfoil_frame.kerf_text_box.delete(1.0, "end")
            self.bot_airfoil_frame.kerf_text_box.delete(1.0, "end")

            # Update the text fields with any available information from the wing
            if wing.name is not None:
                self.wing_setting_frame.name_text.insert(1.0, wing.name)
                ret_val = wing.name
            if wing.span is not None:
                self.wing_setting_frame.span_text.insert(1.0, wing.span)
            if wing.washout is not None:
                self.wing_setting_frame.washout_text.insert(1.0, wing.washout)
            if wing.sweep is not None:
                self.wing_setting_frame.sweep_text.insert(1.0, wing.sweep)
            if wing.root_chord is not None:
                self.top_airfoil_frame.chord_text_box.insert(1.0, wing.root_chord)
                self.bot_airfoil_frame.chord_text_box.insert(1.0, wing.tip_chord)
                self.top_airfoil_frame.kerf_text_box.insert(1.0, wing.root_kerf)
                self.bot_airfoil_frame.kerf_text_box.insert(1.0, wing.tip_kerf)

            align = 1 if wing.align_with_le else 0
            gen_lr = 1 if wing.symmetric else 0

            self.wing_setting_frame.align_led_var.set(align)
            self.wing_setting_frame.gen_lr_var.set(gen_lr)
            self.wing_setting_frame.check_gen_lr()
            self.wing_setting_frame.check_align_led()

            self.wing_setting_frame.cnc_machine_option_menu.configure(
                values=self.wing_setting_frame.get_machine_options())

            if wing.root_airfoil_tag is not None:
                self.top_airfoil_frame.airfoil_option_menu.set(wing.root_airfoil_tag)
                self.bot_airfoil_frame.airfoil_option_menu.set(wing.tip_airfoil_tag)
                self.top_airfoil_frame.plot_airfoil('manual')
                self.bot_airfoil_frame.plot_airfoil('manual')
            else:
                self.top_airfoil_frame.plot_airfoil('manual')
                self.bot_airfoil_frame.plot_airfoil('manual')

        return ret_val

    def get_curr_name(self):
        ret_val = None

        if self.curr_selected is not None:
            # Wing settings have not been saved by this point, so grab the name directly out of the text box field
            ret_val = self.wing_setting_frame.name_text.get(1.0, 'end-1c')

        return ret_val

    def update_from_project(self):
        self.scroll_frame.reset()
        self.reset()

        self.wings = self.root.curr_project.wings

        names = list()
        for item in self.wings:
            names.append(item.name)
        self.scroll_frame.update_from_list(names)

    def reset(self):
        for wing in reversed(self.wings):
            tmp = wing
            self.wings.remove(wing)
            del wing
        del self.wings
        self.wings = list()
        self.curr_selected = None
        self.update_gui(self.curr_selected)

    def slice_selected(self):
        self.save_wing_settings()
        wing = self.wings[self.curr_selected]
        machine = self.root.get_window_instance(window=WindowState.MACHINE_SETUP).get_machine(wing.machine_tag)
        sm.SliceManager.wing_to_gcode(wing=wing, wire_cutter=machine, output_dir=self.root.curr_project.output_dir)

    def slice_all(self):
        orig_sel = self.curr_selected
        for ind in range(len(self.wings)):
            self.curr_selected = ind
            wing = self.wings[self.curr_selected]
            machine = self.root.get_window_instance(window=WindowState.MACHINE_SETUP).get_machine(wing.machine_tag)
            sm.SliceManager.wing_to_gcode(wing=wing, wire_cutter=machine, output_dir=self.root.curr_project.output_dir)
        self.curr_selected = orig_sel


class CADWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(CADWindow, self).__init__(master, WindowState.CAD, root)

        self.grid_rowconfigure(index=0, weight=100)
        self.grid_columnconfigure(index=0, weight=1)
        self.grid_columnconfigure(index=1, weight=8)
        self.grid_propagate(False)

        self.primary_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.primary_frame.grid(row=0, column=1, sticky=tk.NSEW)
        self.primary_frame.grid_columnconfigure(index=0, weight=1)
        self.primary_frame.grid_rowconfigure(index=0, weight=2)
        self.primary_frame.grid_rowconfigure(index=1, weight=1)
        self.primary_frame.grid_propagate(False)
        self.primary_frame.pack_propagate(False)

        self.primary_frame.grid_rowconfigure(index=0, weight=1)
        self.primary_frame.grid_columnconfigure(index=0, weight=1)

        self.set_visibility(False)

        self.scroll_frame = ScrollableSelectionMenu(self, self)
        self.scroll_frame.grid(row=0, column=0, sticky=tk.NSEW)

        self._create_top()
        self._create_bot()

        self.cad_parts = list()

    def set_visibility(self, visible):
        if visible:
            self.primary_frame.grid()
        else:
            self.primary_frame.grid_remove()

    def _create_top(self):
        self.top_frame = tk.Frame(self.primary_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.top_frame.grid(row=0, column=0, sticky=tk.NSEW)
        self.top_frame.grid_rowconfigure(index=0, weight=1)
        self.top_frame.grid_rowconfigure(index=1, weight=4)
        self.top_frame.grid_rowconfigure(index=2, weight=1)
        self.top_frame.grid_columnconfigure(index=0, weight=1)
        self.top_frame.grid_propagate(False)

        self.top_top_frame = tk.Frame(self.top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.top_top_frame.grid(row=0, column=0, sticky=tk.NSEW)
        self.top_top_frame.grid_rowconfigure(index=0, weight=1)
        self.top_top_frame.grid_columnconfigure(index=0, weight=3)
        self.top_top_frame.grid_columnconfigure(index=1, weight=2)
        self.top_top_frame.grid_columnconfigure(index=2, weight=1)
        self.top_top_frame.grid_propagate(False)

        self.name_frame = tk.LabelFrame(self.top_top_frame, text='Name:', background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.name_frame.grid(row=0, column=0, sticky=tk.NSEW)
        self.name_frame.pack_propagate(False)

        self.name_text = tk.Text(self.name_frame)
        self.name_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 2, PrimaryStyle.GENERAL_PADDING / 4),
                            padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.stl_frame = tk.LabelFrame(self.top_top_frame, text='File:', background=PrimaryStyle.SECONDARY_COLOR,
                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                       fg=PrimaryStyle.FONT_COLOR)
        self.stl_frame.grid(row=0, column=1, sticky=tk.NSEW)
        self.stl_frame.grid_rowconfigure(index=0, weight=1)
        self.stl_frame.grid_columnconfigure(index=0, weight=1)
        self.stl_frame.grid_propagate(False)

        self.selected_stl = tk.StringVar()
        self.selected_stl.set('Select File')

        self.stl_menu = ttk.Combobox(self.stl_frame, textvariable=self.selected_stl,
                                     values=self.get_file_options())
        self.stl_menu.grid(row=0, column=0, sticky=tk.NSEW, pady=(PrimaryStyle.GENERAL_PADDING / 2,
                                                                  PrimaryStyle.GENERAL_PADDING / 4),
                           padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.units_frame = tk.LabelFrame(self.top_top_frame, text='File Units:',
                                         background=PrimaryStyle.SECONDARY_COLOR,
                                         highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                         highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                         fg=PrimaryStyle.FONT_COLOR)
        self.units_frame.grid(row=0, column=2, sticky=tk.NSEW)
        self.units_frame.grid_rowconfigure(index=0, weight=1)
        self.units_frame.grid_columnconfigure(index=0, weight=1)
        self.units_frame.grid_propagate(False)

        self.selected_units = tk.StringVar()
        self.selected_units.set('Select Units')

        self.unit_menu = ttk.Combobox(self.units_frame, textvariable=self.selected_units,
                                      values=['mm', 'cm', 'm', 'in'])
        self.unit_menu.grid(row=0, column=0, sticky=tk.NSEW, pady=(PrimaryStyle.GENERAL_PADDING / 2,
                                                                   PrimaryStyle.GENERAL_PADDING / 4),
                            padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.mid_frame = tk.Frame(self.top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.mid_frame.grid(row=1, column=0, sticky=tk.NSEW)
        self.mid_frame.grid_columnconfigure(index=0, weight=1)
        self.mid_frame.grid_columnconfigure(index=1, weight=5)
        self.mid_frame.grid_rowconfigure(index=0, weight=1)
        self.mid_frame.grid_propagate(False)

        self.workpiece_frame = CADWindow.WorkpieceWindow(self.mid_frame, root=self,
                                                         background=PrimaryStyle.SECONDARY_COLOR,
                                                         highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                         highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.workpiece_frame.grid(row=0, column=0, sticky=tk.NSEW)

        self.setting_frame = CADWindow.SettingsWindow(self.mid_frame, root=self,
                                                      background=PrimaryStyle.SECONDARY_COLOR,
                                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.setting_frame.grid(row=0, column=1, sticky=tk.NSEW)

    def _create_bot(self):
        self.bot_frame = tk.Frame(self.primary_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.bot_frame.grid(row=1, column=0, sticky=tk.NSEW)
        self.bot_frame.grid_columnconfigure(index=0, weight=1)
        self.bot_frame.grid_rowconfigure(index=0, weight=1)

        self.stl_prev_window_label_frame = tk.LabelFrame(self.bot_frame, text='Slice Preview:',
                                                         background=PrimaryStyle.SECONDARY_COLOR,
                                                         highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                         highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                         fg=PrimaryStyle.FONT_COLOR)
        self.stl_prev_window_label_frame.grid(row=0, column=0, sticky=tk.NSEW)
        self.stl_prev_window_label_frame.grid_columnconfigure(index=0, weight=1)
        self.stl_prev_window_label_frame.grid_rowconfigure(index=0, weight=1)

        self.stl_prev_window = CADWindow.STLPreviewWindow(self.stl_prev_window_label_frame, root=self,
                                                          background=PrimaryStyle.SECONDARY_COLOR,
                                                          highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                          highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.stl_prev_window.grid(row=0, column=0, sticky=tk.NSEW)

    def get_file_options(self):
        files = list()
        for i in range(1, 4):
            files.append('STL%s' % i)
        for i in range(4, 8):
            files.append('OBJ%s' % i)
        return files

    def update_from_project(self):
        raise NotImplementedError('update_from_project not Implemented for WindowType: %s' % self.window_type)

    def add_item(self):
        raise NotImplementedError('add_item not Implemented for WindowType: %s' % self.window_type)

    def delete_item(self, index):
        raise NotImplementedError('delete_item not Implemented for WindowType: %s' % self.window_type)

    def update_gui(self, index):
        raise NotImplementedError('update_gui not Implemented for WindowType: %s' % self.window_type)

    def get_curr_name(self):
        raise NotImplementedError('get_curr_name not Implemented for WindowType: %s' % self.window_type)

    def reset(self):
        raise NotImplementedError('reset not Implemented for WindowType: %s' % self.window_type)

    class WorkpieceWindow(tk.Frame):
        def __init__(self, master, root, **kwargs):
            super(CADWindow.WorkpieceWindow, self).__init__(master, **kwargs)
            self.root = root

            self.grid_rowconfigure(index=0, weight=1)
            self.grid_columnconfigure(index=0, weight=1)

            self.workpiece_frame = tk.LabelFrame(self, text='Workpiece:', background=PrimaryStyle.SECONDARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                 fg=PrimaryStyle.FONT_COLOR)
            self.workpiece_frame.grid(row=0, column=0, sticky=tk.NSEW,
                                      padx=PrimaryStyle.GENERAL_PADDING / 2,
                                      pady=PrimaryStyle.GENERAL_PADDING / 2)

    class SettingsWindow(tk.Frame):
        def __init__(self, master, root, **kwargs):
            super(CADWindow.SettingsWindow, self).__init__(master, **kwargs)
            self.root = root

            self.grid_rowconfigure(index=0, weight=1)
            self.grid_columnconfigure(index=0, weight=1)

            self.settings_frame = tk.LabelFrame(self, text='Settings:', background=PrimaryStyle.SECONDARY_COLOR,
                                                highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                fg=PrimaryStyle.FONT_COLOR)
            self.settings_frame.grid(row=0, column=0, sticky=tk.NSEW,
                                     padx=PrimaryStyle.GENERAL_PADDING / 2,
                                     pady=PrimaryStyle.GENERAL_PADDING / 2)

    class STLPreviewWindow(tk.Frame):
        def __init__(self, master, root, **kwargs):
            super(CADWindow.STLPreviewWindow, self).__init__(master, **kwargs)
            self.root = root

            self.plots = list()

            self.grid_rowconfigure(index=0, weight=1)
            self.grid_columnconfigure(index=0, weight=1)

            self.canvas_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                         highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                         highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.canvas_frame.grid(row=0, column=0, sticky=tk.NSEW, columnspan=2)
            self.canvas_frame.grid_rowconfigure(index=0, weight=29)
            self.canvas_frame.grid_rowconfigure(index=1, weight=1)
            self.canvas_frame.grid_columnconfigure(index=0, weight=1)
            self.canvas_frame.grid_propagate(False)
            self.canvas_frame.pack_propagate(False)

            self.primary_canvas = tk.Canvas(self.canvas_frame,
                                            background=PrimaryStyle.PRIMARY_COLOR)
            self.primary_canvas.grid(row=0, column=0, sticky=tk.NSEW)
            self.primary_canvas.pack_propagate(False)
            self.primary_canvas.grid_propagate(False)

            self.scroll_window = tk.Frame(self.primary_canvas,
                                          background=PrimaryStyle.PRIMARY_COLOR)

            self.primary_canvas.create_window(0, 0, window=self.scroll_window, anchor=tk.NW)

            self.h_scroll_bar = tk.Scrollbar(self.canvas_frame, orient=tk.HORIZONTAL, command=self.primary_canvas.yview,
                                             bg=PrimaryStyle.PRIMARY_COLOR)
            self.h_scroll_bar.grid(row=1, column=0, sticky=tk.NSEW)
            self.h_scroll_bar.lift(self.scroll_window)

            self.primary_canvas.config(yscrollcommand=self.h_scroll_bar.set,
                                       scrollregion=self.primary_canvas.bbox("all"))

        def add_plots(self, cross_sections):
            for cross_section in cross_sections:
                self.plots.append(PlotWindow(self.primary_canvas, self, background=PrimaryStyle.PRIMARY_COLOR,
                                             highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                             highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS))
                self.plots[-1].pack(side=tk.LEFT, fill=None, anchor=tk.CENTER)
                self.plots[-1].plot(callback=cross_section.plot_gui(PrimaryStyle.FONT_COLOR, PrimaryStyle.TETRARY_COLOR,
                                                                    PrimaryStyle.QUATERNARY_COLOR))

        def delete_plots(self):
            pass


class DatabaseWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(DatabaseWindow, self).__init__(master, WindowState.DATABASE, root)

        self.label = ttk.Label(master=self, text='Database')
        self.label.grid()

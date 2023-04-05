import copy
import logging
import os
import threading
import timeit
import tkinter as tk
import tkinter.filedialog as filedialog
import tkinter.ttk as ttk
import webbrowser

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import g_code.command_library as cl
import geometry.complex as cm
import geometry.parser as gp
import geometry.primative as prim
import geometry.spatial_manipulation as spm
import project_manager as pm
import slicer.slice_manager as sm
import slicer.wire_cutter as wc
from util.util_functions import is_float
from .styles import PrimaryStyle

USERS_GUIDE_LINK = r'https://docs.google.com/document/d/1Y9rKHuZB0Q6IvmGO-PNEoXhAPTkg2_eH27yfsKHy7n8/edit?usp=sharing'


def create_thread_callback(func):
    def callback():
        threading.Thread(target=func).start()

    return callback


class WindowState(object):
    HOME = 0
    MACHINE_SETUP = 1
    WING = 2
    CAD = 3
    DATABASE = 4
    RANGE = range(0, 5)


class GeneralSettings(object):
    LABEL_FRAME_SETTINGS = {
        'background': PrimaryStyle.SECONDARY_COLOR,
        'highlightbackground': PrimaryStyle.HL_BACKGROUND_COL,
        'highlightthickness': PrimaryStyle.HL_BACKGROUND_THICKNESS,
        'fg': PrimaryStyle.FONT_COLOR
    }

    FRAME_SETTINGS = {
        'background': PrimaryStyle.SECONDARY_COLOR,
        'highlightbackground': PrimaryStyle.HL_BACKGROUND_COL,
        'highlightthickness': PrimaryStyle.HL_BACKGROUND_THICKNESS
    }


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
        if len(names) >= 1:
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
            self.root.save_settings()
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
    def __init__(self, master, root, width, height, **kwargs):
        super(PlotWindow, self).__init__(master, width=width, height=height, **kwargs)
        self.root = root

        self.pack_propagate(False)
        self.grid_propagate(False)

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
    """
    :param str title:
    :param int width:
    :param int height:
    :param pm.ProjectManager project_manager:
    """

    def __init__(self, title, width, height, project_manager):
        super(MainWindow, self).__init__(baseName=title)
        self.project_manager = project_manager

        self.logger = logging.getLogger(__name__)

        self.width = width
        self.height = height
        self.window_title = title
        self.title(title)

        self.curr_project = self.project_manager.default_project()

        self.geometry("%sx%s" % (self.width, self.height))

        self.active_window = WindowState.HOME
        self.grid_rowconfigure(index=0, weight=600)
        self.grid_rowconfigure(index=1, weight=3)
        self.grid_columnconfigure(index=0, weight=1)

        self.primary_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR)
        self.primary_frame.grid_configure(column=0, row=0, sticky=tk.NSEW)
        # self.primary_frame.pack(side='top', fill='both', expand=True)
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
        s = ttk.Style()
        s.theme_use('clam')
        s.configure('Horizontal.TProgressbar', foreground=PrimaryStyle.TERTIARY_COLOR,
                    background=PrimaryStyle.TERTIARY_COLOR)
        self.progress_bar = ttk.Progressbar(self, orient='horizontal', mode='indeterminate', length=width - 5)
        self.progress_bar.grid(row=1, column=0, sticky=tk.NSEW)

        self.switch_embedded_window(WindowState.HOME)

        self.resizable(False, False)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_close(self):
        self.project_manager.save_ini_file()
        self.exit()

    def get_window_instance(self, window):
        return self.embedded_windows[window]

    def switch_embedded_window(self, window_enum):
        self.embedded_windows[self.active_window].save_settings()
        self.active_window = window_enum
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
        self.embedded_windows[WindowState.WING].save_settings()
        self.embedded_windows[WindowState.MACHINE_SETUP].save_settings()
        self.embedded_windows[WindowState.CAD].save_settings()
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
        self.project_manager.update_project_list(self.curr_project)

    def load_project(self):
        self.update()
        file_path = filedialog.askopenfilename(title="Open Project", master=self,
                                               filetypes=[('Project', '*.proj')])
        if os.path.exists(file_path):
            self._load_project(file_path)

    def load_project_from_file_path(self, file_path):
        self.update()
        self._load_project(file_path)

    def _load_project(self, file_path):
        self.curr_project = pm.Project.load_project(file_path)
        self.title(self.window_title + ' Project: %s' % self.curr_project.name)
        self.embedded_windows[WindowState.WING].update_from_project()
        self.embedded_windows[WindowState.WING].wings = copy.deepcopy(self.curr_project.wings)
        self.embedded_windows[WindowState.MACHINE_SETUP].update_from_project()
        self.embedded_windows[WindowState.MACHINE_SETUP].machines = copy.deepcopy(self.curr_project.machines)
        self.embedded_windows[WindowState.CAD].update_from_project()
        self.embedded_windows[WindowState.CAD].cad_parts = copy.deepcopy(self.curr_project.cad_parts)
        self.embedded_windows[WindowState.DATABASE].update_from_project()

        self.project_manager.update_project_list(self.curr_project)

    def exit(self):
        self.destroy()

    def _create_menu_bar(self):
        file_menu = tk.Menu(self.menu_bar, tearoff=0)
        file_menu.add_command(label="New Project", command=self.new_project)
        file_menu.add_command(label="Save Project", command=self.save_project)
        file_menu.add_command(label="Load Project", command=self.load_project)
        file_menu.add_command(label="Exit", command=self.on_close)
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
        help_menu.add_command(label="Users Guide", command=lambda: webbrowser.open(USERS_GUIDE_LINK))
        help_menu.add_command(label="Update")
        self.menu_bar.add_cascade(label="Help", menu=help_menu)

        self.config(menu=self.menu_bar)


class EmbeddedWindow(tk.Frame):
    def __init__(self, master, window_type, root):
        super(EmbeddedWindow, self).__init__(master, background=PrimaryStyle.PRIMARY_COLOR)
        self.window_type = window_type
        self.root = root

    @staticmethod
    def manage_checkbox_color(check_box, var):
        val = var.get()
        if val == 1:
            check_box.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
        else:
            check_box.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)

    def save_settings(self):
        pass

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

        self.news_label = tk.Label(self.left_bot_frame, text='News:', bg=PrimaryStyle.TERTIARY_COLOR,
                                   relief=tk.SOLID, borderwidth=1, justify=tk.LEFT,
                                   fg=PrimaryStyle.FONT_COLOR)
        self.news_label.pack(fill=tk.X, anchor=tk.NW, expand=False)

        self.news_box = tk.Text(self.left_bot_frame, background=PrimaryStyle.SECONDARY_COLOR, width=0,
                                fg=PrimaryStyle.FONT_COLOR)
        log = "TODO: Change log"
        self.news_box.insert('end', log)
        self.news_box.config(wrap=tk.WORD)
        self.news_box.pack(fill=tk.BOTH, anchor=tk.NW, expand=True)

        self.proj_list = list()
        self.proj_ref = dict()
        self.proj_var = tk.StringVar(value=self.proj_list)

        self.proj_label = tk.Label(self.right_frame, text='Recent Projects:', bg=PrimaryStyle.TERTIARY_COLOR,
                                   fg=PrimaryStyle.FONT_COLOR,
                                   borderwidth=1, relief=tk.SOLID)
        self.proj_label.pack(fill=tk.X, anchor=tk.NW, expand=False)
        self.recent_projects = tk.Listbox(self.right_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                          listvariable=self.proj_var,
                                          fg=PrimaryStyle.FONT_COLOR)
        self.recent_projects.bind('<Double-1>', self.load_recent_project)
        self.recent_projects.pack(fill=tk.BOTH, anchor=tk.CENTER, expand=True, padx=0, pady=0)

    def load_recent_project(self, event):
        ps = self.recent_projects.curselection()
        selection = ps[0]
        self.root.load_project_from_file_path(self.proj_ref[self.proj_list[selection]])

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

        self.kerf_text = None
        self.logger = logging.getLogger(__name__)

        self.machines = list()
        self.curr_selected = None
        # self.machines.append(wc.WireCutter(name='short', wire_length=245, max_height=300.0, max_depth=980,
        #                                    max_speed=150.0, min_speed=30, release_height=80.0, start_height=10.0,
        #                                    start_depth=20.0, dynamic_tension=True))
        # self.machines[-1].set_dynamic_tension_motor_letter('V')
        # self.machines[-1].reverse_dynamic_tension(True)
        # self.machines[-1].dynamic_tension_feed_comp = True
        #
        # self.machines.append(wc.WireCutter(name='baseline', wire_length=640, max_height=300.0, max_depth=980,
        #                                    max_speed=150.0, min_speed=50, release_height=80.0, start_height=10.0,
        #                                    start_depth=20.0, dynamic_tension=True))
        # self.machines[-1].set_dynamic_tension_motor_letter('V')
        # self.machines[-1].reverse_dynamic_tension(True)
        # self.machines[-1].dynamic_tension_feed_comp = True
        #
        # self.machines.append(wc.WireCutter(name='extended', wire_length=1000, max_height=300.0, max_depth=980,
        #                                    max_speed=150.0, min_speed=30, release_height=80.0, start_height=10.0,
        #                                    start_depth=20.0, dynamic_tension=True))
        # self.machines[-1].set_dynamic_tension_motor_letter('V')
        # self.machines[-1].reverse_dynamic_tension(True)
        # self.machines[-1].dynamic_tension_feed_comp = True

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
        self.left_frame.grid_rowconfigure(index=0, weight=3)
        self.left_frame.grid_rowconfigure(index=1, weight=1)
        self.left_frame.grid_rowconfigure(index=2, weight=2)
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
        self.name_text.bind("<KeyRelease>", self.update_name)

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
        self.right_top_frame.grid_rowconfigure(index=0, weight=3)
        self.right_top_frame.grid_rowconfigure(index=1, weight=2)
        self.right_top_frame.grid_rowconfigure(index=2, weight=1)
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

        self.inv_time_var = tk.IntVar(self)
        self.inv_time_check = tk.Checkbutton(self.feed_frame, text='Inverse Time', onvalue=1, offvalue=0,
                                             variable=self.inv_time_var, command=self.manage_checkbox_inv_time,
                                             background=PrimaryStyle.SECONDARY_COLOR,
                                             selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                             highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                             highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                             fg=PrimaryStyle.FONT_COLOR)
        self.inv_time_check.grid(row=0, column=0, sticky=tk.NW)

        self.upt_var = tk.IntVar(self)
        self.upt_check = tk.Checkbutton(self.feed_frame, text='Units/Min', onvalue=1, offvalue=0,
                                        variable=self.upt_var, command=self.manage_checkbox_upt,
                                        background=PrimaryStyle.SECONDARY_COLOR,
                                        selectcolor=PrimaryStyle.PRIMARY_COLOR,
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
        self.option_frame.grid_columnconfigure(index=1, weight=1)
        self.option_frame.grid_propagate(False)

        self.default_var = tk.IntVar(self)
        self.default_check = tk.Checkbutton(self.option_frame, text='Default', onvalue=1, offvalue=0,
                                            variable=self.default_var, command=self.manage_checkbox_default,
                                            background=PrimaryStyle.SECONDARY_COLOR,
                                            selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR)
        self.default_check.grid(row=0, column=0, sticky=tk.NW)

        self.dynamic_option_frame = tk.LabelFrame(self.left_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                  fg=PrimaryStyle.FONT_COLOR,
                                                  text='Dynamic Tension')
        self.dynamic_option_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 2,
                                       padx=(PrimaryStyle.GENERAL_PADDING, PrimaryStyle.GENERAL_PADDING / 2))
        self.dynamic_option_frame.grid_rowconfigure(index=0, weight=1)
        self.dynamic_option_frame.grid_rowconfigure(index=1, weight=1)
        self.dynamic_option_frame.grid_rowconfigure(index=2, weight=1)
        self.dynamic_option_frame.grid_columnconfigure(index=0, weight=1)
        self.dynamic_option_frame.grid_columnconfigure(index=1, weight=2)
        self.dynamic_option_frame.grid_propagate(False)

        self.dynamic_var = tk.IntVar(self)
        self.dynamic_check = tk.Checkbutton(self.dynamic_option_frame, text='Enable', onvalue=1, offvalue=0,
                                            variable=self.dynamic_var, command=self.manage_checkbox_dynamic,
                                            background=PrimaryStyle.SECONDARY_COLOR,
                                            selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR)
        self.dynamic_check.grid(row=0, column=0, sticky=tk.NW)

        self.dynamic_rev_var = tk.IntVar(self)
        self.dynamic_rev_check = tk.Checkbutton(self.dynamic_option_frame, text='Reverse', onvalue=1, offvalue=0,
                                                variable=self.dynamic_rev_var, command=self.manage_checkbox_dynamic_rev,
                                                background=PrimaryStyle.SECONDARY_COLOR,
                                                selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                                highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                fg=PrimaryStyle.FONT_COLOR)
        self.dynamic_rev_check.grid(row=1, column=0, sticky=tk.NW)

        self.dynamic_feed_comp_var = tk.IntVar(self)
        self.dynamic_feed_comp_check = tk.Checkbutton(self.dynamic_option_frame, text='Feed Rate Comp', onvalue=1,
                                                      offvalue=0,
                                                      variable=self.dynamic_feed_comp_var,
                                                      command=self.manage_checkbox_dynamic_comp,
                                                      background=PrimaryStyle.SECONDARY_COLOR,
                                                      selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                      fg=PrimaryStyle.FONT_COLOR)
        self.dynamic_feed_comp_check.grid(row=2, column=0, sticky=tk.NW)

        self.d1_frame = tk.LabelFrame(self.dynamic_option_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                      fg=PrimaryStyle.FONT_COLOR,
                                      text='Motor Letter:')
        self.d1_frame.grid(row=0, column=1, rowspan=2, sticky=tk.NSEW, pady=(0, 0),
                           padx=(2, 1))
        self.d1_frame.pack_propagate(False)
        self.d1_text = tk.Text(self.d1_frame)
        self.d1_text.insert('end', 'V')
        self.d1_text.pack(pady=PrimaryStyle.GENERAL_PADDING / 4, padx=PrimaryStyle.GENERAL_PADDING / 2)

        # ==============================================================================================================
        # Bottom Frame
        self.bot_left_frame = tk.Frame(self.left_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.bot_left_frame.grid(row=2, column=0, sticky=tk.NSEW)
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

        # Handle the asset being packaged into an EXE, or still being local if running via interpreter
        self.gantry_photo_path = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                                              'assets', 'gui', 'gantries.png'))
        self.logger.debug('Gantry Photo Path: %s', self.gantry_photo_path)

        self.gantry_photo = tk.PhotoImage(
            file=self.gantry_photo_path if os.path.exists(self.gantry_photo_path) else None, width=900, height=900)
        self.gantry_photo = self.gantry_photo.subsample(2)
        self.photo_label = tk.Label(self.right_frame, image=self.gantry_photo)
        self.photo_label.grid(row=0, column=0, sticky=tk.NSEW)

    def save_settings(self):
        if self.curr_selected is not None:
            name = self.name_text.get(1.0, "end-1c")

            wire_length = get_float(self.wire_text.get(1.0, "end-1c"))
            max_height = get_float(self.height_text.get(1.0, "end-1c"))
            max_depth = get_float(self.depth_text.get(1.0, "end-1c"))
            max_speed = get_float(self.fast_text.get(1.0, "end-1c"))
            min_speed = get_float(self.slow_text.get(1.0, "end-1c"))
            x1 = self.x1_text.get(1.0, "end-1c")
            x2 = self.x2_text.get(1.0, "end-1c")
            y1 = self.y1_text.get(1.0, "end-1c")
            y2 = self.y2_text.get(1.0, "end-1c")
            axis_def = x1 + "{:.6f} " + y1 + "{:.6f} " + x2 + "{:.6f} " + y2 + "{:.6f}"
            startup_gcode = self.gcode_text_field.get(1.0, "end-1c")

            dynamic_tension_ena = self.dynamic_var.get()
            dynamic_tension_rev = self.dynamic_rev_var.get()
            dynamic_tension_letter = self.d1_text.get(1.0, "end-1c")
            dynamic_tension_feed_comp = self.dynamic_feed_comp_var.get()

            inv_time_setting = self.inv_time_var.get()
            default = self.default_var.get()

            machine = self.machines[self.curr_selected]
            machine.name = name
            machine.wire_length = wire_length
            machine.max_height = max_height
            machine.max_depth = max_depth
            machine.max_speed = max_speed
            machine.min_speed = min_speed
            machine.axis_def = axis_def
            machine.start_up_gcode = startup_gcode
            machine.dynamic_tension = True if dynamic_tension_ena == 1 else False
            machine.dynamic_tension_reverse = True if dynamic_tension_rev == 1 else False
            machine.dynamic_tension_feed_comp = True if dynamic_tension_feed_comp == 1 else False
            machine.dynamic_tension_motor_letter = dynamic_tension_letter
            machine.default = default

            machine.feed_rate_mode = cl.GCodeCommands.FeedRate.INVERSE_TIME if inv_time_setting == 1 \
                else cl.GCodeCommands.FeedRate.UNITS_PER_MINUTE

    def update_from_project(self):
        self.scroll_frame.reset()
        self.reset()

        self.machines = self.root.curr_project.machines

        names = list()
        for item in self.machines:
            names.append(item.name)
        self.scroll_frame.update_from_list(names)

    def add_item(self):
        self.machines.append(wc.WireCutter(name='', wire_length=None, max_height=None, max_speed=None, min_speed=None,
                                           max_depth=None))

    def delete_item(self, index):
        tmp = self.machines[index]
        self.machines.remove(tmp)
        del tmp

    def update_gui(self, index):
        # Index is only none if we are removing an entry, do not save machine settings if we are deleting
        ret_val = None
        if index is None:
            self.set_visibility(False)
        else:
            self.save_settings()
            self.curr_selected = index
            self.logger.info('gui wing index: %s', index)
            machine = self.machines[self.curr_selected]

            # Clear the text fields
            self.name_text.delete(1.0, "end")
            self.wire_text.delete(1.0, "end")
            self.height_text.delete(1.0, "end")
            self.depth_text.delete(1.0, "end")
            self.fast_text.delete(1.0, "end")
            self.slow_text.delete(1.0, "end")
            self.x1_text.delete(1.0, "end")
            self.x2_text.delete(1.0, "end")
            self.y1_text.delete(1.0, "end")
            self.y2_text.delete(1.0, "end")
            self.gcode_text_field.delete(1.0, "end")
            self.d1_text.delete(1.0, "end")
            self.default_var.set(0)
            self.inv_time_var.set(0)
            self.upt_var.set(0)
            self.dynamic_var.set(0)
            self.dynamic_feed_comp_var.set(0)
            self.dynamic_rev_var.set(0)

            if machine.name is not None:
                self.name_text.insert(1.0, machine.name)
                ret_val = machine.name
            if machine.wire_length is not None:
                self.wire_text.insert(1.0, machine.wire_length)
            if machine.max_height is not None:
                self.height_text.insert(1.0, machine.max_height)
            if machine.max_depth is not None:
                self.depth_text.insert(1.0, machine.max_depth)
            if machine.max_speed is not None:
                self.fast_text.insert(1.0, machine.max_speed)
            if machine.min_speed is not None:
                self.slow_text.insert(1.0, machine.min_speed)
            if machine.axis_def is not None:
                splits = machine.axis_def.split(' ')
                x1 = splits[0][0:1]
                x2 = splits[2][0:1]
                y1 = splits[1][0:1]
                y2 = splits[3][0:1]
                self.x1_text.insert(1.0, x1)
                self.x2_text.insert(1.0, x2)
                self.y1_text.insert(1.0, y1)
                self.y2_text.insert(1.0, y2)
            if machine.start_up_gcode is not None:
                self.gcode_text_field.insert(1.0, machine.start_up_gcode)
            self.dynamic_var.set(1 if machine.dynamic_tension else 0)
            self.dynamic_rev_var.set(1 if machine.dynamic_tension_reverse else 0)
            self.dynamic_feed_comp_var.set(1 if machine.dynamic_tension_feed_comp else 0)
            if machine.dynamic_tension_motor_letter is not None:
                self.d1_text.insert(1.0, machine.dynamic_tension_motor_letter)

            if machine.feed_rate_mode is not None:
                if machine.feed_rate_mode == cl.GCodeCommands.FeedRate.INVERSE_TIME:
                    self.inv_time_var.set(1)
                else:
                    self.upt_var.set(1)

            if machine.default:
                self.default_var.set(1)

            self.manage_checkbox_inv_time()
            self.manage_checkbox_upt()
            self.manage_checkbox_dynamic_comp()
            self.manage_checkbox_dynamic_rev()
            self.manage_checkbox_dynamic()
            self.manage_checkbox_default()

        return ret_val

    def get_curr_name(self):
        ret_val = None

        if self.curr_selected is not None:
            # machine settings might not have been saved by this point, so grab the name directly out of the text box
            # field
            ret_val = self.name_text.get(1.0, 'end-1c')

        return ret_val

    def update_name(self, event):
        self.scroll_frame.update_curr_name(name=self.name_text.get(1.0, "end-1c"))

    def reset(self):
        for machine in reversed(self.machines):
            self.machines.remove(machine)
            del machine
        del self.machines
        self.machines = list()
        self.curr_selected = None
        self.update_gui(self.curr_selected)

    def manage_checkbox_inv_time(self):
        self.manage_checkbox_color(self.inv_time_check, self.inv_time_var)
        if self.inv_time_var.get() == 1:
            self.upt_var.set(0)
            self.manage_checkbox_upt()

    def manage_checkbox_upt(self):
        self.manage_checkbox_color(self.upt_check, self.upt_var)
        if self.upt_var.get() == 1:
            self.inv_time_var.set(0)
            self.manage_checkbox_inv_time()

    def manage_checkbox_default(self):
        self.manage_checkbox_color(self.default_check, self.default_var)

    def manage_checkbox_dynamic(self):
        self.manage_checkbox_color(self.dynamic_check, self.dynamic_var)

    def manage_checkbox_dynamic_rev(self):
        self.manage_checkbox_color(self.dynamic_rev_check, self.dynamic_rev_var)

    def manage_checkbox_dynamic_comp(self):
        self.manage_checkbox_color(self.dynamic_feed_comp_check, self.dynamic_feed_comp_var)


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
        self.left_top_frame.grid_propagate(False)

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
        self.spar_frame.grid(row=1, column=0, sticky=tk.NSEW)

        # ==========================================================================================================
        # Slicer buttons
        self.slice_btn_frame = tk.Frame(self.right_top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.slice_btn_frame.grid(row=2, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING / 4,
                                  pady=PrimaryStyle.GENERAL_PADDING / 4)
        self.slice_btn_frame.grid_rowconfigure(index=0, weight=1)
        self.slice_btn_frame.grid_columnconfigure(index=0, weight=6)
        self.slice_btn_frame.grid_columnconfigure(index=1, weight=8)
        self.slice_btn_frame.grid_propagate(False)

        self.slice_btn = tk.Button(self.slice_btn_frame, text='Slice\nSelected', command=self.slice_selected)
        self.slice_btn.grid(row=0, column=0, sticky=tk.NSEW, padx=(PrimaryStyle.GENERAL_PADDING / 4, 1),
                            pady=PrimaryStyle.GENERAL_PADDING / 4)

        self.slice_all_btn = tk.Button(self.slice_btn_frame, text='Slice\nAll', command=self.slice_all)
        self.slice_all_btn.grid(row=0, column=1, sticky=tk.NSEW, padx=(1, PrimaryStyle.GENERAL_PADDING / 4),
                                pady=PrimaryStyle.GENERAL_PADDING / 4)

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
            airfoils.extend(self.root.main_window.project_manager.airfoils.keys())
            return airfoils

        def get_airfoil_data(self, key):
            ret_val = None
            if key in self.root.main_window.curr_project.database.airfoils:
                ret_val = self.root.main_window.curr_project.database.airfoils[key]
            elif key in self.root.main_window.project_manager.airfoils:
                ret_val = self.root.main_window.project_manager.airfoils[key]
            return ret_val

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
                    data=self._transform_airfoil(self.get_airfoil_data(airfoil_selection)))
                airfoil.plot_points_2d_gui(plot_1, PrimaryStyle.FONT_COLOR, PrimaryStyle.TERTIARY_COLOR)

                holes = self.root.wings[self.root.curr_selected].root_holes if self.position == 0 else self.root.wings[
                    self.root.curr_selected].tip_holes

                if holes is not None:
                    for hole in holes:
                        airfoil_hole = gp.Dat(
                            data=self._transform_hole(hole))
                        airfoil_hole.plot_points_2d_gui(plot_1, PrimaryStyle.FONT_COLOR, PrimaryStyle.TERTIARY_COLOR,
                                                        markersize=1)

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

        def _transform_hole(self, hole):
            chord = self.chord_text_box.get(1.0, 'end-1c')
            chord = float(chord) if is_float(chord) else 1

            washout = self.root.wing_setting_frame.washout_text.get(1.0, 'end-1c')
            washout = float(washout) if is_float(washout) else 0

            sweep = self.root.wing_setting_frame.sweep_text.get(1.0, 'end-1c')
            sweep = float(sweep) if is_float(sweep) else 0

            span = self.root.wing_setting_frame.span_text.get(1.0, 'end-1c')
            span = float(span) if is_float(span) else 1

            chord_offset = hole.chord_start * chord
            airfoil_selection = self.airfoil_option_menu.get()
            airfoil = copy.deepcopy(self.get_airfoil_data(airfoil_selection))
            spm.PointManip.Transform.scale(airfoil, [chord, chord, 1])
            point_1, point_2 = prim.GeometricFunctions.get_points_with_chord_offset(airfoil, chord_offset)
            if point_1['y'] > point_2['y']:
                point_top = point_1
                point_bot = point_2
            else:
                point_top = point_2
                point_bot = point_1
            # plt.scatter([point_top['x'], point_bot['x']], [point_top['y'], point_bot['y']],color='C5')
            hole.build_points(chord, point_top, point_bot)

            points = copy.deepcopy(hole.get_path())

            # spm.PointManip.Transform.scale(points, [chord, chord, 0])

            if self.position == 1:
                spm.PointManip.Transform.rotate(points, [0.0, 0.0, np.deg2rad(washout)])
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

            self.wash_sweep_frame = tk.Frame(self.wing_field_frame,
                                             background=PrimaryStyle.SECONDARY_COLOR,
                                             highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                             highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS, )
            self.wash_sweep_frame.grid(row=2, column=0, sticky=tk.NSEW,
                                       pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.wash_sweep_frame.grid_rowconfigure(index=0, weight=1)
            self.wash_sweep_frame.grid_columnconfigure(index=0, weight=1)
            self.wash_sweep_frame.grid_columnconfigure(index=1, weight=1)

            self.washout_label_frame = tk.LabelFrame(self.wash_sweep_frame, text='Washout (deg):',
                                                     background=PrimaryStyle.SECONDARY_COLOR,
                                                     highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                     highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                     fg=PrimaryStyle.FONT_COLOR, width=14)
            self.washout_label_frame.grid(row=0, column=0, sticky=tk.NSEW,
                                          padx=PrimaryStyle.GENERAL_PADDING / 2,
                                          pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.washout_label_frame.pack_propagate(False)

            self.washout_text = tk.Text(self.washout_label_frame)
            self.washout_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 4,
                                   padx=PrimaryStyle.GENERAL_PADDING / 2)
            self.washout_text.bind("<KeyRelease>", self.root.update_airfoil_plots)
            self.washout_text.bind("<FocusOut>", self.root.update_airfoil_plots)
            self.washout_text.bind("<FocusIn>", self.root.update_airfoil_plots)

            self.sweep_label_frame = tk.LabelFrame(self.wash_sweep_frame, text='Sweep (deg):',
                                                   background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                   fg=PrimaryStyle.FONT_COLOR, width=14)
            self.sweep_label_frame.grid(row=0, column=1, sticky=tk.NSEW,
                                        padx=(0, PrimaryStyle.GENERAL_PADDING / 2),
                                        pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.sweep_label_frame.pack_propagate(False)

            self.sweep_text = tk.Text(self.sweep_label_frame)
            self.sweep_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 4,
                                 padx=PrimaryStyle.GENERAL_PADDING / 2)
            self.sweep_text.bind("<KeyRelease>", self.root.update_airfoil_plots)
            self.sweep_text.bind("<FocusOut>", self.root.update_airfoil_plots)
            self.sweep_text.bind("<FocusIn>", self.root.update_airfoil_plots)

            self.wire_frame = tk.Frame(self.wing_field_frame,
                                       background=PrimaryStyle.SECONDARY_COLOR,
                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS, )
            self.wire_frame.grid(row=3, column=0, sticky=tk.NSEW,
                                 padx=PrimaryStyle.GENERAL_PADDING / 2,
                                 pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.wire_frame.grid_rowconfigure(index=0, weight=1)
            self.wire_frame.grid_columnconfigure(index=0, weight=1)
            self.wire_frame.grid_columnconfigure(index=1, weight=1)
            self.wire_frame.grid_columnconfigure(index=2, weight=1)

            self.start_height_lf = tk.LabelFrame(self.wire_frame, text='St H(mm):',
                                                 background=PrimaryStyle.SECONDARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                 fg=PrimaryStyle.FONT_COLOR, width=14)
            self.start_height_lf.grid(row=0, column=0, sticky=tk.NSEW,
                                      pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.start_height_lf.pack_propagate(False)

            self.start_height = tk.Text(self.start_height_lf)
            self.start_height.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 4,
                                   padx=(PrimaryStyle.GENERAL_PADDING / 2, 0))

            self.start_depth_lf = tk.LabelFrame(self.wire_frame, text='St D(mm):',
                                                background=PrimaryStyle.SECONDARY_COLOR,
                                                highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                fg=PrimaryStyle.FONT_COLOR, width=14)
            self.start_depth_lf.grid(row=0, column=1, sticky=tk.NSEW,
                                     padx=(PrimaryStyle.GENERAL_PADDING / 2, 0),
                                     pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.start_depth_lf.pack_propagate(False)

            self.start_depth = tk.Text(self.start_depth_lf)
            self.start_depth.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 4,
                                  padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.release_height_lf = tk.LabelFrame(self.wire_frame, text='Rel H(mm):',
                                                   background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                   fg=PrimaryStyle.FONT_COLOR, width=14)
            self.release_height_lf.grid(row=0, column=2, sticky=tk.NSEW,
                                        padx=(PrimaryStyle.GENERAL_PADDING / 2, 0),
                                        pady=(0, PrimaryStyle.GENERAL_PADDING / 4))
            self.release_height_lf.pack_propagate(False)

            self.release_height = tk.Text(self.release_height_lf)
            self.release_height.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 4,
                                     padx=PrimaryStyle.GENERAL_PADDING / 2)

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
            # print('check align led set to: %s' % self.align_led_var.get())
            if self.align_led_var.get() == 1:
                self.align_led_edge.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
            else:
                self.align_led_edge.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)

        def check_gen_lr(self):
            # self.gen_lr_var.set(not self.gen_lr_var.get())
            # print('check gen lr set to: %s' % self.gen_lr_var.get())
            if self.gen_lr_var.get() == 1:
                self.gen_left_right.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
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
            self.grid_propagate(False)

            self.spar_frame = tk.LabelFrame(self, text='Advanced', background=PrimaryStyle.SECONDARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR)
            self.spar_frame.grid(row=0, column=0, sticky=tk.NSEW,
                                 padx=PrimaryStyle.GENERAL_PADDING / 2,
                                 pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.spar_frame.grid_rowconfigure(index=0, weight=1)
            self.spar_frame.grid_rowconfigure(index=1, weight=9)
            self.spar_frame.grid_columnconfigure(index=0, weight=1)
            self.spar_frame.grid_propagate(False)

            self.spar_addition_frame = tk.Frame(self.spar_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                                highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.spar_addition_frame.grid(row=0, column=0, sticky=tk.NSEW)
            self.spar_addition_frame.grid_rowconfigure(index=0, weight=1)
            self.spar_addition_frame.grid_columnconfigure(index=0, weight=8)
            self.spar_addition_frame.grid_columnconfigure(index=1, weight=1)
            self.spar_addition_frame.grid_propagate(False)

            self.selected_spar = tk.StringVar()
            self.selected_spar.set('')

            self.spar_option_menu = ttk.Combobox(self.spar_addition_frame, textvariable=self.selected_spar,
                                                 values=[])
            self.spar_option_menu.bind('<<ComboboxSelected>>', self.update)
            self.spar_option_menu.grid(row=0, column=0, sticky=tk.NSEW, padx=(4, 4), pady=1)
            self.spar_option_menu.grid_propagate(False)

            self.add_spar_button = tk.Button(self.spar_addition_frame, text='Add Hole', command=self.add_hole)
            self.add_spar_button.grid(row=0, column=1, sticky=tk.NSEW, padx=(1, 4), pady=1)
            self.add_spar_button.grid_propagate(False)

            self.spar_def_frame = tk.Frame(self.spar_frame, **GeneralSettings.FRAME_SETTINGS)
            self.spar_def_frame.grid(row=1, column=0, sticky=tk.NSEW)
            self.spar_def_frame.grid_rowconfigure(index=0, weight=1)
            self.spar_def_frame.grid_rowconfigure(index=1, weight=69)
            self.spar_def_frame.grid_columnconfigure(index=0, weight=1)
            self.spar_def_frame.grid_propagate(False)
            self.spar_def_frame.pack_propagate(False)

            self.selected_spar_type = tk.StringVar()
            self.selected_spar_type.set('Rectangle')

            self.selected_spar_type = ttk.Combobox(self.spar_def_frame, textvariable=self.selected_spar_type,
                                                   values=['Rectangle', 'Circular', 'Cavity'], )
            self.selected_spar_type.bind('<<ComboboxSelected>>', self.select_spar_window)
            self.selected_spar_type.grid(row=0, column=0, sticky=tk.NSEW, padx=(4, 4), pady=1)
            self.selected_spar_type.grid_remove()

            self.option_frame_rect = self.RectangleSpar(self.spar_def_frame, self)
            self.option_frame_rect.grid(row=1, column=0, sticky=tk.NSEW)
            self.option_frame_rect.set_visibility(False)
            self.option_frame_circle = self.CircularSpar(self.spar_def_frame, self)
            self.option_frame_circle.grid(row=1, column=0, sticky=tk.NSEW)
            self.option_frame_circle.set_visibility(False)
            self.option_frame_cavity = self.CavityHole(self.spar_def_frame, self)
            self.option_frame_cavity.grid(row=1, column=0, sticky=tk.NSEW)
            self.option_frame_cavity.set_visibility(False)

        def update(self, event=None):
            if 'Hole' in self.spar_option_menu.get():
                curr_wing = self.root.wings[self.root.curr_selected]

                if curr_wing.hole_info is None:
                    curr_wing.hole_info = list()

                if self.spar_option_menu.get() == '':
                    self.selected_spar_type.set('')
                    self.selected_spar_type.grid_remove()
                else:
                    self.selected_spar_type.grid()

                self.option_frame_cavity.update_gui()
                self.option_frame_circle.update_gui()
                self.option_frame_rect.update_gui()
                super(WingWindow.SparWindow, self).update()

        def update_available_holes(self):
            self.spar_option_menu.configure(values=list())
            curr_wing = self.root.wings[self.root.curr_selected]
            if hasattr(curr_wing, 'hole_info') and curr_wing.hole_info is not None:
                for ind in range(0, len(curr_wing.hole_info)):
                    self.spar_option_menu.configure(values=list(self.spar_option_menu['values']) + [
                        'Hole%s' % (len(list(self.spar_option_menu['values'])) + 1)])

        def add_hole(self):
            self.spar_option_menu.configure(values=list(self.spar_option_menu['values']) + [
                'Hole%s' % (len(list(self.spar_option_menu['values'])) + 1)])

            if self.spar_option_menu.get() == '':
                self.spar_option_menu.set(list(self.spar_option_menu['values'])[0])
                self.spar_option_menu.update()
                self.selected_spar_type.grid()

        def select_spar_window(self, event):
            window_type = WingWindow.SparWindow.SparWindowTypes.from_str(self.selected_spar_type.get())

            self.option_frame_rect.set_visibility(False)
            self.option_frame_circle.set_visibility(False)
            self.option_frame_cavity.set_visibility(False)
            if window_type == self.SparWindowTypes.RECTANGLE:
                self.option_frame_rect.set_visibility(True)
            elif window_type == self.SparWindowTypes.CIRCULAR:
                self.option_frame_circle.set_visibility(True)
            elif window_type == self.SparWindowTypes.CAVITY:
                self.option_frame_cavity.set_visibility(True)
            self.update()

        def delete_item(self):
            curr_index = int(self.spar_option_menu.get()[4:]) - 1
            if len(list(self.spar_option_menu['values'])) == 1:
                self.spar_option_menu['values'] = ['']
                self.spar_option_menu.set('')
            else:
                self.spar_option_menu['values'] = ['Hole%s' % i for i in
                                                   range(1, len(list(self.spar_option_menu['values'])))]

            self.root.wings[self.root.curr_selected].delete_hole(curr_index)
            self.update()

        class SparWindowTypes(object):
            RECTANGLE = 0
            CIRCULAR = 1
            CAVITY = 2

            @staticmethod
            def from_str(value):
                ret_val = None
                if value == 'Rectangle':
                    ret_val = WingWindow.SparWindow.SparWindowTypes.RECTANGLE
                if value == 'Circular':
                    ret_val = WingWindow.SparWindow.SparWindowTypes.CIRCULAR
                if value == 'Cavity':
                    ret_val = WingWindow.SparWindow.SparWindowTypes.CAVITY
                return ret_val

        class SparInfo(object):
            def __init__(self):
                self.spar_type = None
                self.chord_start = None
                self.chord_stop = None
                self.height = None
                self.width = None
                self.angle = None
                self.offset = None
                self.start_angle = None
                self.num_points = None
                self.radius = None
                self.thickness = None

        class SparWindowSubtype(tk.Frame):
            def __init__(self, master, root, **kwargs):
                super(WingWindow.SparWindow.SparWindowSubtype, self).__init__(master, **kwargs)
                self.pack_propagate(False)
                self.root = root

            def set_visibility(self, visible):
                if visible:
                    self.grid()
                else:
                    self.grid_remove()

            def save_spar_to_wing(self):
                raise NotImplementedError('Must be implemented by inherited class')

            def delete_selected_spar(self):
                curr_index = int(self.root.spar_option_menu.get()[4:]) - 1
                curr_wing = self.root.root.wings[self.root.root.curr_selected]
                del curr_wing.hole_info[curr_index]

                self.root.selected_spar_type.set('')
                self.root.select_spar_window('manual')
                self.root.delete_item()

            def update_gui(self):
                raise NotImplementedError('Must be implemented by inherited class')

        class RectangleSpar(SparWindowSubtype):
            def __init__(self, master, root, **kwargs):
                super(WingWindow.SparWindow.RectangleSpar, self).__init__(master, root, **kwargs)

                for ind in range(4):
                    self.grid_rowconfigure(index=ind, weight=1)
                self.grid_columnconfigure(index=0, weight=1)
                self.grid_columnconfigure(index=1, weight=1)
                self.grid_columnconfigure(index=2, weight=1)

                self.chord_start_frame = tk.LabelFrame(self, text='Chord Start(%)', font=("Arial", 8),
                                                       **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.chord_start_frame.grid(row=0, column=0, columnspan=3, sticky=tk.NSEW)
                self.chord_start_text = tk.Text(self.chord_start_frame)
                self.chord_start_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                           padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.chord_start_frame.pack_propagate(False)

                self.height_frame = tk.LabelFrame(self, text='Height(mm)', font=("Arial", 8),
                                                  **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.height_frame.grid(row=1, column=0, columnspan=1, sticky=tk.NSEW)
                self.height_text = tk.Text(self.height_frame)
                self.height_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                      padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.height_frame.pack_propagate(False)

                self.width_frame = tk.LabelFrame(self, text='Width(mm)', font=("Arial", 8),
                                                 **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.width_frame.grid(row=1, column=1, columnspan=1, sticky=tk.NSEW)
                self.width_text = tk.Text(self.width_frame)
                self.width_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                     padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.width_frame.pack_propagate(False)

                self.angle_frame = tk.LabelFrame(self, text='angle(deg)', font=("Arial", 8),
                                                 **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.angle_frame.grid(row=1, column=2, columnspan=1, sticky=tk.NSEW)
                self.angle_text = tk.Text(self.angle_frame)
                self.angle_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                     padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.angle_frame.pack_propagate(False)

                self.offset_frame = tk.LabelFrame(self, text='offset(mm)', font=("Arial", 8),
                                                  **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.offset_frame.grid(row=2, column=0, columnspan=1, sticky=tk.NSEW)
                self.offset_text = tk.Text(self.offset_frame)
                self.offset_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                      padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.offset_frame.pack_propagate(False)

                self.start_angle_frame = tk.LabelFrame(self, text='Start Angle(deg)', font=("Arial", 8),
                                                       **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.start_angle_frame.grid(row=2, column=1, columnspan=1, sticky=tk.NSEW)
                self.start_angle_text = tk.Text(self.start_angle_frame)
                self.start_angle_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                           padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.start_angle_frame.pack_propagate(False)

                self.num_points_frame = tk.LabelFrame(self, text='Num Points', font=("Arial", 8),
                                                      **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.num_points_frame.grid(row=2, column=2, columnspan=1, sticky=tk.NSEW)
                self.num_points_text = tk.Text(self.num_points_frame)
                self.num_points_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                          padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.num_points_frame.pack_propagate(False)

                self.button_frame = tk.Frame(self, **GeneralSettings.FRAME_SETTINGS)
                self.button_frame.grid(row=3, column=0, columnspan=3, sticky=tk.NSEW)
                self.button_frame.grid_columnconfigure(index=0, weight=1)
                self.button_frame.grid_columnconfigure(index=1, weight=1)
                self.button_frame.grid_rowconfigure(index=0, weight=1)

                self.save = tk.Frame(self.button_frame, **GeneralSettings.FRAME_SETTINGS)
                self.save.grid(row=0, column=0, sticky=tk.NSEW)
                self.save.grid_columnconfigure(index=0, weight=1)
                self.save.grid_rowconfigure(index=0, weight=1)
                self.save.grid_propagate(False)

                self.save_button = tk.Button(self.save, text='Save', command=self.save_spar_to_wing)
                self.save_button.grid(row=0, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                      padx=PrimaryStyle.GENERAL_PADDING / 8)

                self.delete = tk.Frame(self.button_frame, **GeneralSettings.FRAME_SETTINGS)
                self.delete.grid(row=0, column=1, sticky=tk.NSEW)
                self.delete.grid_columnconfigure(index=0, weight=1)
                self.delete.grid_rowconfigure(index=0, weight=1)
                self.delete.grid_propagate(False)
                self.delete_button = tk.Button(self.delete, text='Delete', command=self.delete_selected_spar)
                self.delete_button.grid(row=0, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                        padx=PrimaryStyle.GENERAL_PADDING / 8)

            def save_spar_to_wing(self):
                curr_wing = self.root.root.wings[self.root.root.curr_selected]
                tmp_chord_start = self.chord_start_text.get("1.0", "end-1c")
                tmp_height = self.height_text.get("1.0", "end-1c")
                tmp_width = self.width_text.get("1.0", "end-1c")
                tmp_angle = self.angle_text.get("1.0", "end-1c")
                tmp_offset = self.offset_text.get("1.0", "end-1c")
                tmp_start_angle = self.start_angle_text.get("1.0", "end-1c")
                tmp_num_points = self.num_points_text.get("1.0", "end-1c")

                try:
                    chord_start = float(tmp_chord_start) / 100.0
                    height = float(tmp_height)
                    width = float(tmp_width)
                    angle = float(tmp_angle)
                    offset = float(tmp_offset)
                    start_angle = float(tmp_start_angle)
                    num_points = int(tmp_num_points)

                    curr_index = int(self.root.spar_option_menu.get()[4:]) - 1

                    spar_info = WingWindow.SparWindow.SparInfo()
                    spar_info.spar_type = WingWindow.SparWindow.SparWindowTypes.RECTANGLE
                    spar_info.chord_start = chord_start
                    spar_info.height = height
                    spar_info.width = width
                    spar_info.angle = angle
                    spar_info.offset = offset
                    spar_info.start_angle = start_angle
                    spar_info.num_points = num_points

                    if not hasattr(curr_wing, 'hole_info') or curr_wing.hole_info is None:
                        curr_wing.hole_info = list()

                    if (curr_index == 0 and curr_wing.root_holes is None) or (curr_index >= len(curr_wing.root_holes)):

                        curr_wing.add_hole_root(
                            cm.WingSegment.RectangularHole(chord_start, height, width, angle, offset,
                                                           start_angle, num_points))
                        curr_wing.add_hole_tip(cm.WingSegment.RectangularHole(chord_start, height, width, angle, offset,
                                                                              start_angle, num_points))
                        curr_wing.hole_info.append(spar_info)
                    else:
                        curr_wing.replace_hole_root(curr_index,
                                                    cm.WingSegment.RectangularHole(chord_start, height, width, angle,
                                                                                   offset,
                                                                                   start_angle, num_points))
                        curr_wing.replace_hole_tip(curr_index,
                                                   cm.WingSegment.RectangularHole(chord_start, height, width, angle,
                                                                                  offset,
                                                                                  start_angle, num_points))
                        curr_wing.hole_info[curr_index] = spar_info

                    self.root.root.save_settings()
                    self.root.root.top_airfoil_frame.plot_airfoil('manual')
                    self.root.root.bot_airfoil_frame.plot_airfoil('manual')
                except ValueError:
                    self.root.root.logger.info('Warning: One of the Spar settings is not convertible to a number')

            def delete_selected_spar(self):
                super(WingWindow.SparWindow.RectangleSpar, self).delete_selected_spar()

            def update_gui(self):
                curr_index = int(self.root.spar_option_menu.get()[4:]) - 1
                self.chord_start_text.delete(1.0, "end")
                self.height_text.delete(1.0, "end")
                self.width_text.delete(1.0, "end")
                self.angle_text.delete(1.0, "end")
                self.offset_text.delete(1.0, "end")
                self.start_angle_text.delete(1.0, "end")
                self.num_points_text.delete(1.0, "end")

                curr_wing = self.root.root.wings[self.root.root.curr_selected]
                if curr_wing.hole_info is not None and len(curr_wing.hole_info) > curr_index:
                    spar_info = curr_wing.hole_info[curr_index]
                    if spar_info.spar_type == WingWindow.SparWindow.SparWindowTypes.RECTANGLE:
                        self.chord_start_text.insert(1.0, spar_info.chord_start*100.0)
                        self.height_text.insert(1.0, spar_info.height)
                        self.width_text.insert(1.0, spar_info.width)
                        self.angle_text.insert(1.0, spar_info.angle)
                        self.offset_text.insert(1.0, spar_info.offset)
                        self.start_angle_text.insert(1.0, spar_info.start_angle)
                        self.num_points_text.insert(1.0, spar_info.num_points)

        class CircularSpar(SparWindowSubtype):
            def __init__(self, master, root, **kwargs):
                super(WingWindow.SparWindow.CircularSpar, self).__init__(master, root, **kwargs)

                for ind in range(4):
                    self.grid_rowconfigure(index=ind, weight=1)
                self.grid_columnconfigure(index=0, weight=1)
                self.grid_columnconfigure(index=1, weight=1)

                self.chord_start_frame = tk.LabelFrame(self, text='Chord Start(%)', font=("Arial", 8),
                                                       **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.chord_start_frame.grid(row=0, column=0, columnspan=1, sticky=tk.NSEW)
                self.chord_start_text = tk.Text(self.chord_start_frame)
                self.chord_start_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                           padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.chord_start_frame.pack_propagate(False)

                self.radius_frame = tk.LabelFrame(self, text='Radius(mm)', font=("Arial", 8),
                                                  **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.radius_frame.grid(row=0, column=1, columnspan=1, sticky=tk.NSEW)
                self.radius_text = tk.Text(self.radius_frame)
                self.radius_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                      padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.radius_frame.pack_propagate(False)

                self.angle_frame = tk.LabelFrame(self, text='Angle(deg)', font=("Arial", 8),
                                                 **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.angle_frame.grid(row=1, column=0, columnspan=1, sticky=tk.NSEW)
                self.angle_text = tk.Text(self.angle_frame)
                self.angle_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                     padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.angle_frame.pack_propagate(False)

                self.offset_frame = tk.LabelFrame(self, text='Offset(mm)', font=("Arial", 8),
                                                  **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.offset_frame.grid(row=1, column=1, columnspan=1, sticky=tk.NSEW)
                self.offset_text = tk.Text(self.offset_frame)
                self.offset_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                      padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.offset_frame.pack_propagate(False)

                self.start_angle_frame = tk.LabelFrame(self, text='Start Angle(mm)', font=("Arial", 8),
                                                       **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.start_angle_frame.grid(row=2, column=0, columnspan=1, sticky=tk.NSEW)
                self.start_angle_text = tk.Text(self.start_angle_frame)
                self.start_angle_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                           padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.start_angle_frame.pack_propagate(False)

                self.num_points_frame = tk.LabelFrame(self, text='Num Points', font=("Arial", 8),
                                                      **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.num_points_frame.grid(row=2, column=1, columnspan=1, sticky=tk.NSEW)
                self.num_points_text = tk.Text(self.num_points_frame)
                self.num_points_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                          padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.num_points_frame.pack_propagate(False)

                self.save = tk.Frame(self, **GeneralSettings.FRAME_SETTINGS)
                self.save.grid(row=3, column=0, sticky=tk.NSEW)
                self.save.grid_columnconfigure(index=0, weight=1)
                self.save.grid_rowconfigure(index=0, weight=1)
                self.save.grid_propagate(False)

                self.save_button = tk.Button(self.save, text='Save', command=self.save_spar_to_wing)
                self.save_button.grid(row=0, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                      padx=PrimaryStyle.GENERAL_PADDING / 8)

                self.delete = tk.Frame(self, **GeneralSettings.FRAME_SETTINGS)
                self.delete.grid(row=3, column=1, sticky=tk.NSEW)
                self.delete.grid_columnconfigure(index=0, weight=1)
                self.delete.grid_rowconfigure(index=0, weight=1)
                self.delete.grid_propagate(False)
                self.delete_button = tk.Button(self.delete, text='Delete', command=self.delete_selected_spar)
                self.delete_button.grid(row=0, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                        padx=PrimaryStyle.GENERAL_PADDING / 8)

            def save_spar_to_wing(self):
                curr_wing = self.root.root.wings[self.root.root.curr_selected]
                tmp_chord_start = self.chord_start_text.get("1.0", "end-1c")
                tmp_radius = self.radius_text.get("1.0", "end-1c")
                tmp_angle = self.angle_text.get("1.0", "end-1c")
                tmp_offset = self.offset_text.get("1.0", "end-1c")
                tmp_start_angle = self.start_angle_text.get("1.0", "end-1c")
                tmp_num_points = self.num_points_text.get("1.0", "end-1c")

                try:
                    chord_start = float(tmp_chord_start) / 100.0
                    radius = float(tmp_radius)
                    angle = float(tmp_angle)
                    offset = float(tmp_offset)
                    start_angle = float(tmp_start_angle)
                    num_points = int(tmp_num_points)

                    curr_index = int(self.root.spar_option_menu.get()[4:]) - 1

                    spar_info = WingWindow.SparWindow.SparInfo()
                    spar_info.spar_type = WingWindow.SparWindow.SparWindowTypes.CIRCULAR
                    spar_info.chord_start = chord_start
                    spar_info.radius = radius
                    spar_info.angle = angle
                    spar_info.offset = offset
                    spar_info.start_angle = start_angle
                    spar_info.num_points = num_points

                    if not hasattr(curr_wing, 'hole_info') or curr_wing.hole_info is None:
                        curr_wing.hole_info = list()

                    if (curr_index == 0 and curr_wing.root_holes is None) or (curr_index >= len(curr_wing.root_holes)):

                        curr_wing.add_hole_root(cm.WingSegment.CircleHole(chord_start, radius, angle, offset,
                                                                          start_angle, num_points))
                        curr_wing.add_hole_tip(cm.WingSegment.CircleHole(chord_start, radius, angle, offset,
                                                                         start_angle, num_points))
                        curr_wing.hole_info.append(spar_info)
                    else:
                        curr_wing.replace_hole_root(curr_index,
                                                    cm.WingSegment.CircleHole(chord_start, radius, angle, offset,
                                                                              start_angle, num_points))
                        curr_wing.replace_hole_tip(curr_index,
                                                   cm.WingSegment.CircleHole(chord_start, radius, angle, offset,
                                                                             start_angle, num_points))
                        curr_wing.hole_info[curr_index] = spar_info

                    self.root.root.save_settings()
                    self.root.root.top_airfoil_frame.plot_airfoil('manual')
                    self.root.root.bot_airfoil_frame.plot_airfoil('manual')
                except ValueError:
                    self.root.root.logger.info('Warning: One of the Spar settings is not convertible to a number')

            def delete_selected_spar(self):
                super(WingWindow.SparWindow.CircularSpar, self).delete_selected_spar()

            def update_gui(self):
                curr_index = int(self.root.spar_option_menu.get()[4:]) - 1
                self.chord_start_text.delete(1.0, "end")
                self.radius_text.delete(1.0, "end")
                self.angle_text.delete(1.0, "end")
                self.offset_text.delete(1.0, "end")
                self.start_angle_text.delete(1.0, "end")
                self.num_points_text.delete(1.0, "end")

                curr_wing = self.root.root.wings[self.root.root.curr_selected]
                if curr_wing.hole_info is not None and len(curr_wing.hole_info) > curr_index:
                    spar_info = curr_wing.hole_info[curr_index]
                    if spar_info.spar_type == WingWindow.SparWindow.SparWindowTypes.CIRCULAR:
                        self.chord_start_text.insert(1.0, spar_info.chord_start*100.0)
                        self.radius_text.insert(1.0, spar_info.radius)
                        self.angle_text.insert(1.0, spar_info.angle)
                        self.offset_text.insert(1.0, spar_info.offset)
                        self.start_angle_text.insert(1.0, spar_info.start_angle)
                        self.num_points_text.insert(1.0, spar_info.num_points)

        class CavityHole(SparWindowSubtype):
            def __init__(self, master, root, **kwargs):
                super(WingWindow.SparWindow.CavityHole, self).__init__(master, root, **kwargs)

                for ind in range(4):
                    self.grid_rowconfigure(index=ind, weight=1)
                self.grid_columnconfigure(index=0, weight=1)
                self.grid_columnconfigure(index=1, weight=1)

                self.chord_start_frame = tk.LabelFrame(self, text='Chord Start(%)', font=("Arial", 8),
                                                       **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.chord_start_frame.grid(row=0, column=0, columnspan=2, sticky=tk.NSEW)
                self.chord_start_text = tk.Text(self.chord_start_frame)
                self.chord_start_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                           padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.chord_start_frame.pack_propagate(False)

                self.chord_stop_frame = tk.LabelFrame(self, text='Chord Stop(%)', font=("Arial", 8),
                                                      **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.chord_stop_frame.grid(row=1, column=0, columnspan=1, sticky=tk.NSEW)
                self.chord_stop_text = tk.Text(self.chord_stop_frame)
                self.chord_stop_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                          padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.chord_stop_frame.pack_propagate(False)

                self.wall_thickness_frame = tk.LabelFrame(self, text='Wall Thickness(mm)', font=("Arial", 8),
                                                          **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.wall_thickness_frame.grid(row=1, column=1, columnspan=1, sticky=tk.NSEW)
                self.wall_thickness_text = tk.Text(self.wall_thickness_frame)
                self.wall_thickness_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                              padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.wall_thickness_frame.pack_propagate(False)

                self.num_points_frame = tk.LabelFrame(self, text='Num Points', font=("Arial", 8),
                                                      **GeneralSettings.LABEL_FRAME_SETTINGS)
                self.num_points_frame.grid(row=2, column=0, columnspan=2, sticky=tk.NSEW)
                self.num_points_text = tk.Text(self.num_points_frame)
                self.num_points_text.pack(expand=True, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                          padx=PrimaryStyle.GENERAL_PADDING / 8)
                self.num_points_frame.pack_propagate(False)

                self.save = tk.Frame(self, **GeneralSettings.FRAME_SETTINGS)
                self.save.grid(row=3, column=0, sticky=tk.NSEW)
                self.save.grid_columnconfigure(index=0, weight=1)
                self.save.grid_rowconfigure(index=0, weight=1)
                self.save.grid_propagate(False)

                self.save_button = tk.Button(self.save, text='Save', command=self.save_spar_to_wing)
                self.save_button.grid(row=0, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                      padx=PrimaryStyle.GENERAL_PADDING / 8)

                self.delete = tk.Frame(self, **GeneralSettings.FRAME_SETTINGS)
                self.delete.grid(row=3, column=1, sticky=tk.NSEW)
                self.delete.grid_columnconfigure(index=0, weight=1)
                self.delete.grid_rowconfigure(index=0, weight=1)
                self.delete.grid_propagate(False)
                self.delete_button = tk.Button(self.delete, text='Delete', command=self.delete_selected_spar)
                self.delete_button.grid(row=0, column=0, sticky=tk.NSEW, pady=PrimaryStyle.GENERAL_PADDING / 8,
                                        padx=PrimaryStyle.GENERAL_PADDING / 8)

            def save_spar_to_wing(self):
                curr_wing = self.root.root.wings[self.root.root.curr_selected]
                tmp_chord_start = self.chord_start_text.get("1.0", "end-1c")
                tmp_chord_stop = self.chord_stop_text.get("1.0", "end-1c")
                tmp_thickness = self.wall_thickness_text.get("1.0", "end-1c")
                tmp_num_points = self.num_points_text.get("1.0", "end-1c")

                try:  # TODO: Better error handling
                    chord_start = float(tmp_chord_start) / 100.0
                    chord_stop = float(tmp_chord_stop) / 100.0
                    thickness = float(tmp_thickness)
                    num_points = int(tmp_num_points)

                    curr_index = int(self.root.spar_option_menu.get()[4:]) - 1

                    root_foil = self.root.root.top_airfoil_frame.airfoil_option_menu.get()
                    root_foil = copy.deepcopy(
                        self.root.root.top_airfoil_frame.get_airfoil_data(root_foil)) if root_foil != '' else None
                    tip_foil = self.root.root.bot_airfoil_frame.airfoil_option_menu.get()
                    tip_foil = copy.deepcopy(
                        self.root.root.bot_airfoil_frame.get_airfoil_data(tip_foil)) if tip_foil != '' else None

                    spar_info = WingWindow.SparWindow.SparInfo()
                    spar_info.spar_type = WingWindow.SparWindow.SparWindowTypes.CAVITY
                    spar_info.chord_start = chord_start
                    spar_info.chord_stop = chord_stop
                    spar_info.thickness = thickness
                    spar_info.num_points = num_points

                    if not hasattr(curr_wing, 'hole_info') or curr_wing.hole_info is None:
                        curr_wing.hole_info = list()

                    if (curr_index == 0 and curr_wing.root_holes is None) or (curr_index >= len(curr_wing.root_holes)):
                        curr_wing.add_hole_root(cm.WingSegment.CavityHole(chord_start, chord_stop, root_foil, thickness,
                                                                          num_points=num_points))
                        curr_wing.add_hole_tip(cm.WingSegment.CavityHole(chord_start, chord_stop, tip_foil, thickness,
                                                                         num_points=num_points))
                        curr_wing.hole_info.append(spar_info)
                    else:
                        curr_wing.replace_hole_root(curr_index, cm.WingSegment.CavityHole(chord_start, chord_stop,
                                                                                          root_foil, thickness,
                                                                                          num_points=num_points))
                        curr_wing.replace_hole_tip(curr_index, cm.WingSegment.CavityHole(chord_start, chord_stop,
                                                                                         tip_foil, thickness,
                                                                                         num_points=num_points))
                        curr_wing.hole_info[curr_index] = spar_info

                    self.root.root.save_settings()
                    self.root.root.top_airfoil_frame.plot_airfoil('manual')
                    self.root.root.bot_airfoil_frame.plot_airfoil('manual')
                except ValueError:
                    self.root.root.logger.info('Warning: One of the Spar settings is not convertible to a number')

            def delete_selected_spar(self):
                super(WingWindow.SparWindow.CavityHole, self).delete_selected_spar()

            def update_gui(self):
                curr_index = int(self.root.spar_option_menu.get()[4:]) - 1
                self.chord_start_text.delete(1.0, "end")
                self.chord_stop_text.delete(1.0, "end")
                self.wall_thickness_text.delete(1.0, "end")
                self.num_points_text.delete(1.0, "end")

                curr_wing = self.root.root.wings[self.root.root.curr_selected]
                if curr_wing.hole_info is not None and len(curr_wing.hole_info) > curr_index:
                    spar_info = curr_wing.hole_info[curr_index]
                    if spar_info.spar_type == WingWindow.SparWindow.SparWindowTypes.CAVITY:
                        self.chord_start_text.insert(1.0, spar_info.chord_start*100.0)
                        self.chord_stop_text.insert(1.0, spar_info.chord_stop*100.0)
                        self.wall_thickness_text.insert(1.0, spar_info.thickness)
                        self.num_points_text.insert(1.0, spar_info.num_points)

    def save_settings(self):
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

            tmp_start_height = self.wing_setting_frame.start_height.get("1.0", "end-1c")
            tmp_start_depth = self.wing_setting_frame.start_depth.get("1.0", "end-1c")
            tmp_release_height = self.wing_setting_frame.release_height.get("1.0", "end-1c")

            wing.span = get_float(tmp_span)
            wing.washout = get_float(tmp_washout)
            wing.sweep = get_float(tmp_sweep)
            wing.tip_chord = get_float(tmp_tip_chord)
            wing.root_chord = get_float(tmp_root_chord)
            wing.tip_kerf = get_float(tmp_tip_kerf)
            wing.root_kerf = get_float(tmp_root_kerf)
            wing.start_height = get_float(tmp_start_height)
            wing.start_depth = get_float(tmp_start_depth)
            wing.release_height = get_float(tmp_release_height)

            wing.tip_airfoil_tag = self.bot_airfoil_frame.airfoil_option_menu.get()
            wing.root_airfoil_tag = self.top_airfoil_frame.airfoil_option_menu.get()
            if wing.tip_airfoil_tag != 'Select Airfoil':
                wing.tip_airfoil = copy.deepcopy(self.bot_airfoil_frame.get_airfoil_data(wing.tip_airfoil_tag))
            if wing.root_airfoil_tag != 'Select Airfoil':
                wing.root_airfoil = copy.deepcopy(self.top_airfoil_frame.get_airfoil_data(wing.root_airfoil_tag))

            wing.symmetric = True if self.wing_setting_frame.gen_lr_var.get() == 1 else False
            wing.align_with_le = True if self.wing_setting_frame.align_led_var.get() == 1 else False

    def add_item(self):
        self.wings.append(cm.WingSegment(name='', logger=self.logger))

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
            self.save_settings()
            self.curr_selected = index
            self.logger.info('gui wing index: %s', index)
            wing = self.wings[self.curr_selected]

            # Clear the text fields
            self.wing_setting_frame.name_text.delete(1.0, "end")
            self.wing_setting_frame.span_text.delete(1.0, "end")
            self.wing_setting_frame.washout_text.delete(1.0, "end")
            self.wing_setting_frame.sweep_text.delete(1.0, "end")
            self.wing_setting_frame.start_height.delete(1.0, "end")
            self.wing_setting_frame.start_depth.delete(1.0, "end")
            self.wing_setting_frame.release_height.delete(1.0, "end")
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
            if wing.start_height is not None:
                self.wing_setting_frame.start_height.insert(1.0, wing.start_height)
            if wing.start_depth is not None:
                self.wing_setting_frame.start_depth.insert(1.0, wing.start_depth)
            if wing.release_height is not None:
                self.wing_setting_frame.release_height.insert(1.0, wing.release_height)

            align = 1 if wing.align_with_le else 0
            gen_lr = 1 if wing.symmetric else 0

            self.wing_setting_frame.align_led_var.set(align)
            self.wing_setting_frame.gen_lr_var.set(gen_lr)
            self.wing_setting_frame.check_gen_lr()
            self.wing_setting_frame.check_align_led()

            self.wing_setting_frame.cnc_machine_option_menu.configure(
                values=self.wing_setting_frame.get_machine_options())
            if wing.machine_tag is not None and wing.machine_tag in self.wing_setting_frame.get_machine_options():
                self.wing_setting_frame.cnc_machine_option_menu.set(wing.machine_tag)

            if wing.root_airfoil_tag is not None:
                self.top_airfoil_frame.airfoil_option_menu.set(wing.root_airfoil_tag)
                self.bot_airfoil_frame.airfoil_option_menu.set(wing.tip_airfoil_tag)
                self.top_airfoil_frame.plot_airfoil('manual')
                self.bot_airfoil_frame.plot_airfoil('manual')
            else:
                self.top_airfoil_frame.plot_airfoil('manual')
                self.bot_airfoil_frame.plot_airfoil('manual')
            self.spar_frame.update_available_holes()

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
        self.update_airfoil_options()
        self.spar_frame.update_available_holes()

    def reset(self):
        for wing in reversed(self.wings):
            tmp = wing
            self.wings.remove(wing)
            del tmp
        del self.wings
        self.wings = list()
        self.curr_selected = None
        self.update_gui(self.curr_selected)

    def slice_selected(self):
        self.save_settings()
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

    def update_airfoil_options(self):
        options = self.top_airfoil_frame.get_airfoil_options()
        self.top_airfoil_frame.airfoil_option_menu['values'] = options
        self.bot_airfoil_frame.airfoil_option_menu['values'] = options


class CADWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(CADWindow, self).__init__(master, WindowState.CAD, root)

        self.logger = logging.getLogger(__name__)

        self.curr_selected = None
        self.cad_parts = list()

        self.grid_rowconfigure(index=0, weight=100)
        self.grid_columnconfigure(index=0, weight=1)
        self.grid_columnconfigure(index=1, weight=8)
        self.grid_propagate(False)

        self.primary_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.primary_frame.grid(row=0, column=1, sticky=tk.NSEW)
        self.primary_frame.grid_columnconfigure(index=0, weight=1)
        self.primary_frame.grid_rowconfigure(index=0, weight=1)
        self.primary_frame.grid_rowconfigure(index=1, weight=1)
        self.primary_frame.grid_propagate(False)
        self.primary_frame.pack_propagate(False)

        self.set_visibility(False)

        self.scroll_frame = ScrollableSelectionMenu(self, self)
        self.scroll_frame.grid(row=0, column=0, sticky=tk.NSEW)

        self._create_top()
        self._create_bot()

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
        self.top_frame.grid_rowconfigure(index=0, weight=15)
        self.top_frame.grid_rowconfigure(index=1, weight=62)
        self.top_frame.grid_rowconfigure(index=2, weight=16)
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
        self.name_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 4),
                            padx=PrimaryStyle.GENERAL_PADDING / 2)
        self.name_text.bind("<KeyRelease>", self.update_name)
        self.name_text.bind("<FocusOut>", self.update_name)
        self.name_text.bind("<FocusIn>", self.update_name)

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
        self.stl_menu.grid(row=0, column=0, sticky=tk.NSEW, pady=(PrimaryStyle.GENERAL_PADDING / 4,
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
        self.unit_menu.grid(row=0, column=0, sticky=tk.NSEW, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                                   PrimaryStyle.GENERAL_PADDING / 4),
                            padx=PrimaryStyle.GENERAL_PADDING / 2)

        self.mid_frame = tk.Frame(self.top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.mid_frame.grid(row=1, column=0, sticky=tk.NSEW)
        self.mid_frame.grid_columnconfigure(index=0, weight=2)
        self.mid_frame.grid_columnconfigure(index=1, weight=10)
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

        self.bot_frame = tk.Frame(self.top_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.bot_frame.grid(row=2, column=0, sticky=tk.NSEW)
        self.bot_frame.grid_columnconfigure(index=0, weight=1)
        self.bot_frame.grid_rowconfigure(index=0, weight=1)
        self.bot_frame.grid_propagate(False)

        self.slice_frame = CADWindow.SliceWindow(self.bot_frame, root=self,
                                                 background=PrimaryStyle.SECONDARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.slice_frame.grid(row=0, column=0, sticky=tk.NSEW)

    def _create_bot(self):
        self.bot_frame = tk.Frame(self.primary_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.bot_frame.grid(row=1, column=0, sticky=tk.NSEW)
        self.bot_frame.grid_columnconfigure(index=0, weight=1)
        self.bot_frame.grid_rowconfigure(index=0, weight=1)

        self.stl_prev_label_frame_text = tk.StringVar()
        self.stl_prev_label_frame_text.set('Slice Preview:')

        self.stl_prev_window_label_frame = tk.LabelFrame(self.bot_frame, text=self.stl_prev_label_frame_text.get(),
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
        options = list(self.root.curr_project.database.cad_files.keys()) + list(
            self.root.project_manager.cad_files.keys())
        return options

    def update_name(self, event):
        self.scroll_frame.update_curr_name(name=self.name_text.get("1.0", "end-1c"))

    def save_settings(self):
        if self.curr_selected is not None:
            cad_part = self.cad_parts[self.curr_selected]
            cad_part.name = self.name_text.get("1.0", "end-1c")

            cad_part.stl_tag = self.stl_menu.get()
            cad_part.units = self.unit_menu.get()
            cad_part.machine_tag = self.slice_frame.cnc_machine_option_menu.get()

            tmp_wp_length = self.workpiece_frame.length_text.get("1.0", "end-1c")
            tmp_wp_height = self.workpiece_frame.height_text.get("1.0", "end-1c")
            tmp_wp_thickness = self.workpiece_frame.thickness_text.get("1.0", "end-1c")
            tmp_subdivisions = self.setting_frame.subdivision_text.get("1.0", "end-1c")
            tmp_wall_thickness = self.setting_frame.wall_thickness_text.get("1.0", "end-1c")
            tmp_section_gap = self.setting_frame.section_gap_text.get("1.0", "end-1c")

            num_points = self.setting_frame.num_points_text.get(1.0, 'end-1c')

            tmp_kerf = self.setting_frame.kerf_text.get(1.0, 'end-1c')
            tmp_max_kerf = self.setting_frame.max_kerf_text.get(1.0, 'end-1c')

            tmp_axis = 0
            if self.setting_frame.secondary_axis_var.get() == 1:
                tmp_axis = 1
            elif self.setting_frame.tertiary_axis_var.get() == 1:
                tmp_axis = 2

            flip_axis = True if self.setting_frame.flip_axis_var.get() == 1 else False
            open_nose = True if self.setting_frame.open_nose_var.get() == 1 else False
            open_tail = True if self.setting_frame.open_tail_var.get() == 1 else False

            cad_part.wp_length = get_float(tmp_wp_length)
            cad_part.wp_height = get_float(tmp_wp_height)
            cad_part.wp_thickness = get_float(tmp_wp_thickness)
            cad_part.subdivisions = get_int(tmp_subdivisions)
            cad_part.wall_thickness = get_float(tmp_wall_thickness)
            cad_part.section_gap = get_float(tmp_section_gap)
            cad_part.num_points = get_int(num_points)
            cad_part.kerf = get_float(tmp_kerf)
            cad_part.max_kerf = get_float(tmp_max_kerf)
            cad_part.slice_axis = tmp_axis
            cad_part.flip_axis = flip_axis
            cad_part.open_nose = open_nose
            cad_part.open_tail = open_tail
            cad_part.hollow_list = self.setting_frame.advanced_hollow_cntrl_text.get(1.0, 'end-1c')

    def update_from_project(self):
        self.scroll_frame.reset()
        self.reset()

        self.cad_parts = self.root.curr_project.cad_parts

        names = list()
        for item in self.cad_parts:
            names.append(item.name)
        self.scroll_frame.update_from_list(names)

    def add_item(self):
        self.cad_parts.append(sm.CadParts(name='', logger=self.logger))

    def delete_item(self, index):
        tmp = self.cad_parts[index]
        self.cad_parts.remove(tmp)
        del tmp

    def update_gui(self, index):
        # Index is only none if we are removing an entry, do not save wing settings if we are deleting
        ret_val = None
        if index is None:
            self.set_visibility(False)
        else:
            self.save_settings()
            self.curr_selected = index
            self.logger.info('gui cad index: %s', index)
            cad_part = self.cad_parts[self.curr_selected]

            # Clear the text fields
            self.name_text.delete(1.0, "end")
            self.workpiece_frame.length_text.delete(1.0, "end")
            self.workpiece_frame.height_text.delete(1.0, "end")
            self.workpiece_frame.thickness_text.delete(1.0, "end")
            self.setting_frame.subdivision_text.delete(1.0, "end")
            self.setting_frame.wall_thickness_text.delete(1.0, "end")
            self.setting_frame.section_gap_text.delete(1.0, "end")
            self.setting_frame.num_points_text.delete(1.0, "end")
            self.setting_frame.kerf_text.delete(1.0, "end")
            self.setting_frame.max_kerf_text.delete(1.0, "end")
            self.setting_frame.advanced_hollow_cntrl_text.delete(1.0, "end")

            # Update the text fields with any available information from the wing
            if cad_part.name is not None:
                self.name_text.insert(1.0, cad_part.name)
                ret_val = cad_part.name
            if cad_part.wp_length is not None:
                self.workpiece_frame.length_text.insert(1.0, str(cad_part.wp_length))
            if cad_part.wp_height is not None:
                self.workpiece_frame.height_text.insert(1.0, str(cad_part.wp_height))
            if cad_part.wp_thickness is not None:
                self.workpiece_frame.thickness_text.insert(1.0, str(cad_part.wp_thickness))
            if cad_part.subdivisions is not None:
                self.setting_frame.subdivision_text.insert(1.0, str(int(cad_part.subdivisions)))
            if cad_part.wall_thickness is not None:
                self.setting_frame.wall_thickness_text.insert(1.0, str(cad_part.wall_thickness))
            if cad_part.section_gap is not None:
                self.setting_frame.section_gap_text.insert(1.0, str(cad_part.section_gap))
            if hasattr(cad_part, 'num_points') and cad_part.num_points is not None:
                self.setting_frame.num_points_text.insert(1.0, str(cad_part.num_points))
            else:
                self.setting_frame.num_points_text.insert(1.0, str(512))  # Num points defaults to 512
            if hasattr(cad_part, 'kerf') and cad_part.kerf is not None:
                self.setting_frame.kerf_text.insert(1.0, str(cad_part.kerf))
            if hasattr(cad_part, 'max_kerf') and cad_part.max_kerf is not None:
                self.setting_frame.max_kerf_text.insert(1.0, str(cad_part.max_kerf))
            if hasattr(cad_part, 'slice_axis') and cad_part.slice_axis is not None:
                if cad_part.slice_axis == 0:
                    self.setting_frame.primary_axis_var.set(1)
                    self.setting_frame.check_primary()
                elif cad_part.slice_axis == 1:
                    self.setting_frame.secondary_axis_var.set(1)
                    self.setting_frame.check_secondary()
                elif cad_part.slice_axis == 2:
                    self.setting_frame.tertiary_axis_var.set(1)
                    self.setting_frame.check_tertiary()
            if hasattr(cad_part, 'flip_axis') and cad_part.flip_axis is not None:
                if cad_part.flip_axis:
                    self.setting_frame.flip_axis_var.set(1)
                    self.setting_frame.check_flip()
            if hasattr(cad_part, 'open_nose') and cad_part.open_nose is not None:
                if cad_part.open_nose:
                    self.setting_frame.open_nose_var.set(1)
                    self.setting_frame.check_open_nose()
            if hasattr(cad_part, 'open_tail') and cad_part.open_tail is not None:
                if cad_part.open_tail:
                    self.setting_frame.open_tail_var.set(1)
                    self.setting_frame.check_open_tail()
            if hasattr(cad_part, 'hollow_list') and cad_part.hollow_list is not None:
                self.setting_frame.advanced_hollow_cntrl_text.insert(1.0, cad_part.hollow_list)

            self.stl_menu.configure(
                values=self.get_file_options())

            if cad_part.stl_tag is not None:
                self.stl_menu.set(cad_part.stl_tag)
            else:
                self.stl_menu.set('')
            if cad_part.units is not None:
                self.unit_menu.set(cad_part.units)
            else:
                self.unit_menu.set('mm')
            self.slice_frame.cnc_machine_option_menu.configure(
                values=self.slice_frame.get_machine_options())
            if cad_part.machine_tag is not None:
                self.slice_frame.cnc_machine_option_menu.set(cad_part.machine_tag)
            else:
                self.slice_frame.cnc_machine_option_menu.set('')

        return ret_val

    def get_curr_name(self):
        ret_val = None
        if self.curr_selected is not None:
            ret_val = self.name_text.get(1.0, 'end-1c')
        return ret_val

    def reset(self):
        for cad_part in reversed(self.cad_parts):
            tmp = cad_part
            self.cad_parts.remove(tmp)
            del tmp
        del self.cad_parts
        self.cad_parts = list()
        self.curr_selected = None
        self.update_gui(self.curr_selected)

    def update_cad_files(self):
        options = self.get_file_options()
        self.stl_menu['values'] = options

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
                                      pady=(PrimaryStyle.GENERAL_PADDING / 2, 0))
            self.workpiece_frame.grid_columnconfigure(index=0, weight=1)
            self.workpiece_frame.grid_rowconfigure(index=0, weight=1)
            self.workpiece_frame.grid_rowconfigure(index=1, weight=1)
            self.workpiece_frame.grid_rowconfigure(index=2, weight=1)
            self.workpiece_frame.grid_propagate(False)

            self.length_frame = tk.LabelFrame(self.workpiece_frame, text='Length (mm):',
                                              background=PrimaryStyle.SECONDARY_COLOR,
                                              highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                              highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                              fg=PrimaryStyle.FONT_COLOR)
            self.length_frame.grid(row=0, column=0, sticky=tk.NSEW,
                                   padx=PrimaryStyle.GENERAL_PADDING / 2,
                                   pady=PrimaryStyle.GENERAL_PADDING / 2)

            self.length_text = tk.Text(self.length_frame)
            self.length_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                     PrimaryStyle.GENERAL_PADDING / 4),
                                  padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.height_frame = tk.LabelFrame(self.workpiece_frame, text='Height (mm):',
                                              background=PrimaryStyle.SECONDARY_COLOR,
                                              highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                              highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                              fg=PrimaryStyle.FONT_COLOR)
            self.height_frame.grid(row=1, column=0, sticky=tk.NSEW,
                                   padx=PrimaryStyle.GENERAL_PADDING / 2,
                                   pady=PrimaryStyle.GENERAL_PADDING / 2)

            self.height_text = tk.Text(self.height_frame)
            self.height_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                     PrimaryStyle.GENERAL_PADDING / 4),
                                  padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.thickness_frame = tk.LabelFrame(self.workpiece_frame, text='Thickness (mm):',
                                                 background=PrimaryStyle.SECONDARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                 fg=PrimaryStyle.FONT_COLOR)
            self.thickness_frame.grid(row=2, column=0, sticky=tk.NSEW,
                                      padx=PrimaryStyle.GENERAL_PADDING / 2,
                                      pady=PrimaryStyle.GENERAL_PADDING / 2)

            self.thickness_text = tk.Text(self.thickness_frame)
            self.thickness_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                        PrimaryStyle.GENERAL_PADDING / 4),
                                     padx=PrimaryStyle.GENERAL_PADDING / 2)

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
                                     pady=(PrimaryStyle.GENERAL_PADDING / 2, 0))
            self.settings_frame.grid_columnconfigure(index=0, weight=7)
            self.settings_frame.grid_columnconfigure(index=1, weight=5)
            self.settings_frame.grid_columnconfigure(index=2, weight=5)
            self.settings_frame.grid_columnconfigure(index=3, weight=6)
            self.settings_frame.grid_rowconfigure(index=0, weight=1)
            self.settings_frame.grid_rowconfigure(index=1, weight=1)
            self.settings_frame.grid_rowconfigure(index=2, weight=1)
            self.settings_frame.grid_propagate(False)

            self.subdivision_frame = tk.LabelFrame(self.settings_frame, text='Subdivisions:',
                                                   background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                   fg=PrimaryStyle.FONT_COLOR)
            self.subdivision_frame.grid(row=0, column=0, sticky=tk.NSEW,
                                        padx=PrimaryStyle.GENERAL_PADDING / 2,
                                        pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.subdivision_frame.pack_propagate(False)

            self.subdivision_text = tk.Text(self.subdivision_frame)
            self.subdivision_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                          PrimaryStyle.GENERAL_PADDING / 4),
                                       padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.wall_thickness_frame = tk.LabelFrame(self.settings_frame, text='Wall Thickness (mm):',
                                                      background=PrimaryStyle.SECONDARY_COLOR,
                                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                      fg=PrimaryStyle.FONT_COLOR)
            self.wall_thickness_frame.grid(row=2, column=0, sticky=tk.NSEW,
                                           padx=PrimaryStyle.GENERAL_PADDING / 2,
                                           pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.wall_thickness_frame.pack_propagate(False)

            self.wall_thickness_text = tk.Text(self.wall_thickness_frame)
            self.wall_thickness_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                             PrimaryStyle.GENERAL_PADDING / 4),
                                          padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.section_gap_frame = tk.LabelFrame(self.settings_frame, text='Section Gap (mm):',
                                                   background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                   fg=PrimaryStyle.FONT_COLOR)
            self.section_gap_frame.grid(row=1, column=0, sticky=tk.NSEW,
                                        padx=PrimaryStyle.GENERAL_PADDING / 2,
                                        pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.section_gap_frame.pack_propagate(False)

            self.section_gap_text = tk.Text(self.section_gap_frame)
            self.section_gap_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                          PrimaryStyle.GENERAL_PADDING / 4),
                                       padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.kerf_frame = tk.LabelFrame(self.settings_frame, text='Kerf (mm):',
                                            background=PrimaryStyle.SECONDARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR)

            self.kerf_frame.grid(row=1, column=1, sticky=tk.NSEW,
                                 padx=PrimaryStyle.GENERAL_PADDING / 2,
                                 pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.kerf_frame.pack_propagate(False)

            self.kerf_text = tk.Text(self.kerf_frame)
            self.kerf_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                   PrimaryStyle.GENERAL_PADDING / 4),
                                padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.max_kerf_frame = tk.LabelFrame(self.settings_frame, text='Max Kerf (mm):',
                                                background=PrimaryStyle.SECONDARY_COLOR,
                                                highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                fg=PrimaryStyle.FONT_COLOR)
            self.max_kerf_frame.grid(row=2, column=1, sticky=tk.NSEW,
                                     padx=PrimaryStyle.GENERAL_PADDING / 2,
                                     pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.max_kerf_frame.pack_propagate(False)

            self.max_kerf_text = tk.Text(self.max_kerf_frame)
            self.max_kerf_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                       PrimaryStyle.GENERAL_PADDING / 4),
                                    padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.num_points_frame = tk.LabelFrame(self.settings_frame, text='Num Points:',
                                                  background=PrimaryStyle.SECONDARY_COLOR,
                                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                  fg=PrimaryStyle.FONT_COLOR)
            self.num_points_frame.grid(row=0, column=1, sticky=tk.NSEW,
                                       padx=PrimaryStyle.GENERAL_PADDING / 2,
                                       pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.num_points_frame.pack_propagate(False)

            self.num_points_text = tk.Text(self.num_points_frame)
            self.num_points_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                         PrimaryStyle.GENERAL_PADDING / 4),
                                      padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.hollow_cntrl_frame = tk.LabelFrame(self.settings_frame, text='Hollow Settings',
                                                    background=PrimaryStyle.SECONDARY_COLOR,
                                                    highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                    highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                    fg=PrimaryStyle.FONT_COLOR)
            self.hollow_cntrl_frame.grid(row=0, column=2, rowspan=5, sticky=tk.NSEW,
                                         padx=PrimaryStyle.GENERAL_PADDING / 2,
                                         pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.hollow_cntrl_frame.grid_rowconfigure(index=0, weight=1)
            self.hollow_cntrl_frame.grid_rowconfigure(index=1, weight=1)
            self.hollow_cntrl_frame.grid_rowconfigure(index=2, weight=18)
            self.hollow_cntrl_frame.grid_columnconfigure(index=0, weight=1)

            self.open_nose_var = tk.IntVar(self)
            self.open_nose = tk.Checkbutton(self.hollow_cntrl_frame, text='Open Nose', onvalue=1, offvalue=0,
                                            variable=self.open_nose_var, command=self.check_open_nose,
                                            background=PrimaryStyle.SECONDARY_COLOR,
                                            selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR)
            self.open_nose.grid(row=0, column=0, sticky=tk.W)

            self.open_tail_var = tk.IntVar(self)
            self.open_tail = tk.Checkbutton(self.hollow_cntrl_frame, text='Open Tail', onvalue=1, offvalue=0,
                                            variable=self.open_tail_var, command=self.check_open_tail,
                                            background=PrimaryStyle.SECONDARY_COLOR,
                                            selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR)
            self.open_tail.grid(row=1, column=0, sticky=tk.W)

            self.advanced_hollow_cntrl_frame = tk.LabelFrame(self.hollow_cntrl_frame, text='Advanced Hollow List:',
                                                             background=PrimaryStyle.SECONDARY_COLOR,
                                                             highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                             highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                             fg=PrimaryStyle.FONT_COLOR)
            self.advanced_hollow_cntrl_frame.grid(row=2, column=0, sticky=tk.NSEW,
                                                  padx=PrimaryStyle.GENERAL_PADDING / 2,
                                                  pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.advanced_hollow_cntrl_frame.pack_propagate(False)

            self.advanced_hollow_cntrl_text = tk.Text(self.advanced_hollow_cntrl_frame)
            self.advanced_hollow_cntrl_text.pack(expand=True, pady=(PrimaryStyle.GENERAL_PADDING / 4,
                                                                    PrimaryStyle.GENERAL_PADDING / 4),
                                                 padx=PrimaryStyle.GENERAL_PADDING / 2)

            self.axis_cntrl_frame = tk.LabelFrame(self.settings_frame, text='Slice Axis Control',
                                                  background=PrimaryStyle.SECONDARY_COLOR,
                                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                  fg=PrimaryStyle.FONT_COLOR)
            self.axis_cntrl_frame.grid(row=0, column=3, rowspan=5, sticky=tk.NSEW,
                                       padx=PrimaryStyle.GENERAL_PADDING / 2,
                                       pady=PrimaryStyle.GENERAL_PADDING / 2)
            self.axis_cntrl_frame.grid_rowconfigure(index=0, weight=1)
            self.axis_cntrl_frame.grid_rowconfigure(index=1, weight=1)
            self.axis_cntrl_frame.grid_rowconfigure(index=2, weight=1)
            self.axis_cntrl_frame.grid_rowconfigure(index=3, weight=1)
            self.axis_cntrl_frame.grid_columnconfigure(index=0, weight=1)
            self.axis_cntrl_frame.grid_propagate(False)

            self.primary_axis_var = tk.IntVar(self)
            self.primary_axis = tk.Checkbutton(self.axis_cntrl_frame, text='Primary Axis', onvalue=1, offvalue=0,
                                               variable=self.primary_axis_var, command=self.check_primary,
                                               background=PrimaryStyle.SECONDARY_COLOR,
                                               selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                               highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                               highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                               fg=PrimaryStyle.FONT_COLOR)
            self.primary_axis.grid(row=0, column=0, sticky=tk.W)
            self.primary_axis.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
            self.primary_axis_var.set(1)

            self.secondary_axis_var = tk.IntVar(self)
            self.secondary_axis = tk.Checkbutton(self.axis_cntrl_frame, text='Secondary Axis', onvalue=1, offvalue=0,
                                                 variable=self.secondary_axis_var, command=self.check_secondary,
                                                 background=PrimaryStyle.SECONDARY_COLOR,
                                                 selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                 fg=PrimaryStyle.FONT_COLOR)
            self.secondary_axis.grid(row=1, column=0, sticky=tk.W)

            self.tertiary_axis_var = tk.IntVar(self)
            self.tertiary_axis = tk.Checkbutton(self.axis_cntrl_frame, text='Tertiary Axis', onvalue=1, offvalue=0,
                                                variable=self.tertiary_axis_var, command=self.check_tertiary,
                                                background=PrimaryStyle.SECONDARY_COLOR,
                                                selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                                highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                fg=PrimaryStyle.FONT_COLOR)
            self.tertiary_axis.grid(row=2, column=0, sticky=tk.W)

            self.flip_axis_var = tk.IntVar(self)
            self.flip_axis = tk.Checkbutton(self.axis_cntrl_frame, text='Flip Axis', onvalue=1, offvalue=0,
                                            variable=self.flip_axis_var, command=self.check_flip,
                                            background=PrimaryStyle.SECONDARY_COLOR,
                                            selectcolor=PrimaryStyle.PRIMARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                            fg=PrimaryStyle.FONT_COLOR)
            self.flip_axis.grid(row=3, column=0, sticky=tk.W)

        def check_primary(self):
            if self.primary_axis_var.get() == 1:
                self.primary_axis.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
                self.secondary_axis.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)
                self.tertiary_axis.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)
                self.secondary_axis_var.set(0)
                self.tertiary_axis_var.set(0)
            else:
                self.primary_axis.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
                self.primary_axis_var.set(1)

        def check_secondary(self):
            if self.secondary_axis_var.get() == 1:
                self.secondary_axis.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
                self.primary_axis.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)
                self.tertiary_axis.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)
                self.primary_axis_var.set(0)
                self.tertiary_axis_var.set(0)
            else:
                self.secondary_axis.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)
                self.primary_axis.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
                self.primary_axis_var.set(1)

        def check_tertiary(self):
            if self.tertiary_axis_var.get() == 1:
                self.tertiary_axis.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
                self.primary_axis.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)
                self.secondary_axis.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)
                self.primary_axis_var.set(0)
                self.secondary_axis_var.set(0)
            else:
                self.tertiary_axis.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)
                self.primary_axis.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
                self.primary_axis_var.set(1)

        def check_flip(self):
            if self.flip_axis_var.get() == 1:
                self.flip_axis.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
            else:
                self.flip_axis.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)

        def check_open_nose(self):
            if self.open_nose_var.get() == 1:
                self.open_nose.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
            else:
                self.open_nose.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)

        def check_open_tail(self):
            if self.open_tail_var.get() == 1:
                self.open_tail.configure(selectcolor=PrimaryStyle.TERTIARY_COLOR)
            else:
                self.open_tail.configure(selectcolor=PrimaryStyle.PRIMARY_COLOR)

    class SliceWindow(tk.Frame):
        def __init__(self, master, root, **kwargs):
            super(CADWindow.SliceWindow, self).__init__(master, **kwargs)
            self.root = root

            self.grid_rowconfigure(index=0, weight=1)
            self.grid_columnconfigure(index=0, weight=1)

            self.slice_frame = tk.Frame(self, background=PrimaryStyle.SECONDARY_COLOR,
                                        highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                        highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.slice_frame.grid(row=0, column=0, sticky=tk.NSEW,
                                  padx=PrimaryStyle.GENERAL_PADDING / 2,
                                  pady=(0, 0))
            self.slice_frame.grid_rowconfigure(index=0, weight=1)
            self.slice_frame.grid_columnconfigure(index=0, weight=1)
            self.slice_frame.grid_columnconfigure(index=1, weight=1)
            self.slice_frame.grid_columnconfigure(index=2, weight=1)
            self.slice_frame.grid_propagate(False)

            self.sel_cnc_machine_frame = tk.LabelFrame(self.slice_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                       fg=PrimaryStyle.FONT_COLOR,
                                                       text='Select CNC Machine:')
            self.sel_cnc_machine_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, PrimaryStyle.GENERAL_PADDING / 2),
                                            pady=(PrimaryStyle.GENERAL_PADDING / 4, PrimaryStyle.GENERAL_PADDING / 2))
            self.sel_cnc_machine_frame.grid_propagate(False)

            self.selected_machine = tk.StringVar()
            self.selected_machine.set('Select CNC Machine')

            self.cnc_machine_option_menu = ttk.Combobox(self.sel_cnc_machine_frame, textvariable=self.selected_machine,
                                                        values=[])
            self.cnc_machine_option_menu.grid(row=0, column=0, sticky=tk.NSEW, padx=(4, 4), pady=1)

            # ==========================================================================================================
            # Slicer buttons
            self.slice_btn_frame = tk.Frame(self.slice_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.slice_btn_frame.grid(row=0, column=2, sticky=tk.NSEW, padx=(PrimaryStyle.GENERAL_PADDING / 4, 0),
                                      pady=(PrimaryStyle.GENERAL_PADDING / 4, 0))
            self.slice_btn_frame.grid_rowconfigure(index=0, weight=1)
            self.slice_btn_frame.grid_columnconfigure(index=0, weight=6)
            self.slice_btn_frame.grid_columnconfigure(index=1, weight=8)
            self.slice_btn_frame.grid_columnconfigure(index=2, weight=8)
            self.slice_btn_frame.grid_propagate(False)

            self.preview_btn = tk.Button(self.slice_btn_frame, text='Preview\nSelected',
                                         command=create_thread_callback(self.preview_selected))
            self.preview_btn.grid(row=0, column=0, sticky=tk.NSEW, padx=(PrimaryStyle.GENERAL_PADDING / 4, 1),
                                  pady=(PrimaryStyle.GENERAL_PADDING / 4, 0))

            self.slice_btn = tk.Button(self.slice_btn_frame, text='Slice\nSelected',
                                       command=create_thread_callback(self.slice_selected))
            self.slice_btn.grid(row=0, column=1, sticky=tk.NSEW, padx=(PrimaryStyle.GENERAL_PADDING / 4, 1),
                                pady=(PrimaryStyle.GENERAL_PADDING / 4, 0))

            self.slice_all_btn = tk.Button(self.slice_btn_frame, text='Slice\nAll',
                                           command=create_thread_callback(self.slice_all))
            self.slice_all_btn.grid(row=0, column=2, sticky=tk.NSEW, padx=(1, 0),
                                    pady=(PrimaryStyle.GENERAL_PADDING / 4, 0))

        def preview_selected(self):
            self.root.root.progress_bar.start()

            name = self.root.name_text.get(1.0, 'end-1c')
            stl_path = self.root.root.curr_project.database.cad_files[self.root.stl_menu.get()]
            logger = self.root.logger
            output_dir = self.root.root.curr_project.output_dir
            units = self.root.unit_menu.get()
            length = get_float(self.root.workpiece_frame.length_text.get(1.0, 'end-1c'))
            height = get_float(self.root.workpiece_frame.height_text.get(1.0, 'end-1c'))
            thickness = get_float(self.root.workpiece_frame.thickness_text.get(1.0, 'end-1c'))
            workpiece = prim.WorkPiece(length, height, thickness)
            hollow_section_list = self._parse_hollow_list(
                self.root.setting_frame.advanced_hollow_cntrl_text.get(1.0, 'end-1c'))
            open_nose = True if self.root.setting_frame.open_nose_var.get() == 1 else False
            open_tail = True if self.root.setting_frame.open_tail_var.get() == 1 else False
            wall_thickness = get_float(self.root.setting_frame.wall_thickness_text.get(1.0, 'end-1c'))
            subdivisions = int(get_float(self.root.setting_frame.subdivision_text.get(1.0, 'end-1c')))
            flip_axis = True if self.root.setting_frame.flip_axis_var.get() == 1 else False
            slice_axis = 0
            if self.root.setting_frame.secondary_axis_var.get() == 1:
                slice_axis = 1
            elif self.root.setting_frame.tertiary_axis_var.get() == 1:
                slice_axis = 2

            machine = self.root.root.get_window_instance(window=WindowState.MACHINE_SETUP).get_machine(
                self.cnc_machine_option_menu.get())

            num_points = int(get_float(self.root.setting_frame.num_points_text.get(1.0, 'end-1c')))

            start_time = timeit.default_timer()
            cross_section_list = sm.SliceManager.stl_precondition(stl_path=stl_path, logger=logger, units=units,
                                                                  output_dir=output_dir, work_piece=workpiece,
                                                                  hollow_section_list=hollow_section_list,
                                                                  open_nose=open_nose, open_tail=open_tail,
                                                                  wall_thickness=wall_thickness, name=name,
                                                                  subdivisions=subdivisions,
                                                                  wire_cutter=machine,
                                                                  flip_axis=flip_axis, slice_axis=slice_axis,
                                                                  num_points=num_points)
            logger.debug('Preview runtime %ss with %s point resolution', (timeit.default_timer() - start_time),
                         num_points)
            self.root.stl_prev_window_label_frame.update()
            self.root.stl_prev_window.add_plots(cross_section_list)
            self.root.root.progress_bar.stop()

        def slice_selected(self):
            self.root.root.progress_bar.start()

            name = self.root.name_text.get(1.0, 'end-1c')
            stl_path = self.root.root.curr_project.database.cad_files[self.root.stl_menu.get()]
            output_dir = self.root.root.curr_project.output_dir
            units = self.root.unit_menu.get()
            length = get_float(self.root.workpiece_frame.length_text.get(1.0, 'end-1c'))
            height = get_float(self.root.workpiece_frame.height_text.get(1.0, 'end-1c'))
            thickness = get_float(self.root.workpiece_frame.thickness_text.get(1.0, 'end-1c'))
            workpiece = prim.WorkPiece(length, height, thickness)
            hollow_section_list = self._parse_hollow_list(
                self.root.setting_frame.advanced_hollow_cntrl_text.get(1.0, 'end-1c'))
            open_nose = True if self.root.setting_frame.open_nose_var.get() == 1 else False
            open_tail = True if self.root.setting_frame.open_tail_var.get() == 1 else False
            wall_thickness = get_float(self.root.setting_frame.wall_thickness_text.get(1.0, 'end-1c'))
            subdivisions = int(get_float(self.root.setting_frame.subdivision_text.get(1.0, 'end-1c')))
            section_gap = get_float(self.root.setting_frame.section_gap_text.get(1.0, 'end-1c'))
            flip_axis = True if self.root.setting_frame.flip_axis_var.get() == 1 else False
            slice_axis = 0
            if self.root.setting_frame.secondary_axis_var.get() == 1:
                slice_axis = 1
            elif self.root.setting_frame.tertiary_axis_var.get() == 1:
                slice_axis = 2

            kerf = get_float(self.root.setting_frame.kerf_text.get(1.0, 'end-1c'))
            max_kerf = get_float(self.root.setting_frame.max_kerf_text.get(1.0, 'end-1c'))

            machine = self.root.root.get_window_instance(window=WindowState.MACHINE_SETUP).get_machine(
                self.cnc_machine_option_menu.get())
            machine.set_kerf(kerf, max_kerf)

            num_points = int(get_float(self.root.setting_frame.num_points_text.get(1.0, 'end-1c')))

            try:
                cross_section_list = sm.SliceManager.stl_to_gcode(stl_path=stl_path, units=units,
                                                                  output_dir=output_dir, work_piece=workpiece,
                                                                  hollow_section_list=hollow_section_list,
                                                                  open_nose=open_nose, open_tail=open_tail,
                                                                  wall_thickness=wall_thickness, name=name,
                                                                  subdivisions=subdivisions, section_gap=section_gap,
                                                                  wire_cutter=machine,
                                                                  flip_axis=flip_axis, slice_axis=slice_axis,
                                                                  num_points=num_points)
                self.root.stl_prev_window_label_frame.update()
                self.root.stl_prev_window.add_plots(cross_section_list)
            except ValueError:
                pass  # TODO create a warning pop-up window

            self.root.root.progress_bar.stop()

        def slice_all(self):
            self.root.save_settings()
            for cad_part in self.root.cad_parts:
                self.root.root.progress_bar.start()

                name = cad_part.anme
                stl_path = self.root.root.curr_project.database.cad_files[cad_part.stl_tag]
                output_dir = self.root.root.curr_project.output_dir
                units = cad_part.units
                length = cad_part.wp_length
                height = cad_part.wp_height
                thickness = cad_part.wp_thickness
                workpiece = prim.WorkPiece(length, height, thickness)
                hollow_section_list = self._parse_hollow_list(cad_part.hollow_list)
                open_nose = cad_part.open_nose
                open_tail = cad_part.open_tail
                wall_thickness = cad_part.wall_thickness
                subdivisions = cad_part.subdivisions
                section_gap = cad_part.section_gap
                flip_axis = cad_part.flip_axis

                machine = self.root.root.get_window_instance(window=WindowState.MACHINE_SETUP).get_machine(
                    cad_part.machine_tag)

                num_points = cad_part.num_points

                cross_section_list = sm.SliceManager.stl_to_gcode(stl_path=stl_path, units=units,
                                                                  output_dir=output_dir, work_piece=workpiece,
                                                                  hollow_section_list=hollow_section_list,
                                                                  open_nose=open_nose, open_tail=open_tail,
                                                                  wall_thickness=wall_thickness, name=name,
                                                                  subdivisions=subdivisions, section_gap=section_gap,
                                                                  wire_cutter=machine,
                                                                  flip_axis=flip_axis, num_points=num_points)
                self.root.stl_prev_window_label_frame.update()
                self.root.stl_prev_window.add_plots(cross_section_list)
                self.root.root.progress_bar.stop()

        def get_machine_options(self):
            machines = list()
            for machine in self.root.root.get_window_instance(window=WindowState.MACHINE_SETUP).machines:
                machines.append(machine.name)
            return machines

        def _parse_hollow_list(self, hollow_list_text):
            """

            :params tr hollow_list_text:
            :return:
            """
            hollow_section_list = None
            if len(hollow_list_text) > 0:
                hollow_section_list = list()
                splits = hollow_list_text.split(',')
                for split in splits:
                    hollow_section_list.append(int(split))
            return hollow_section_list

    class STLPreviewWindow(tk.Frame):
        def __init__(self, master, root, **kwargs):
            super(CADWindow.STLPreviewWindow, self).__init__(master, **kwargs)
            self.root = root

            self.plots = list()

            self.grid_rowconfigure(index=0, weight=1)
            self.grid_columnconfigure(index=0, weight=1)
            self.grid_propagate(False)
            self.pack_propagate(False)

            self.canvas_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                         highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                         highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.canvas_frame.grid(row=0, column=0, sticky=tk.NSEW, )
            self.canvas_frame.grid_rowconfigure(index=0, weight=59)
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
            self.primary_canvas.update()
            self.block_size = self.primary_canvas.winfo_reqheight()
            self.primary_canvas.create_window(0, 0, window=self.scroll_window, anchor=tk.NW)

            self.h_scroll_bar = tk.Scrollbar(self.canvas_frame, orient=tk.HORIZONTAL, command=self.primary_canvas.xview,
                                             bg=PrimaryStyle.PRIMARY_COLOR, width=11)
            self.h_scroll_bar.grid(row=1, column=0, sticky=tk.NSEW)
            self.h_scroll_bar.lift(self.scroll_window)

            self.primary_canvas.config(xscrollcommand=self.h_scroll_bar.set,
                                       scrollregion=self.primary_canvas.bbox("all"))

            self.scroll_window.bind('<Configure>', self._configure_window)
            self.scroll_window.bind('<Enter>', self._bound_to_mousewheel)
            self.scroll_window.bind('<Leave>', self._unbound_to_mousewheel)

        def _bound_to_mousewheel(self, event):
            self.primary_canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        def _unbound_to_mousewheel(self, event):
            self.primary_canvas.unbind_all("<MouseWheel>")

        def _on_mousewheel(self, event):
            self.primary_canvas.xview_scroll(int(-1 * (event.delta / 120.0)), "units")

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

        def add_plots(self, cross_sections):
            self.delete_plots()
            for cross_section in cross_sections:
                # self.primary_canvas.update()
                self.plots.append(PlotWindow(self.scroll_window, self, width=self.block_size,
                                             height=self.block_size,
                                             background=PrimaryStyle.PRIMARY_COLOR,
                                             highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                             highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS))

                self.plots[-1].pack(side=tk.LEFT, fill=None, anchor=tk.NW)
                self.plots[-1].plot(
                    callback=cross_section.plot_gui(PrimaryStyle.FONT_COLOR, PrimaryStyle.TERTIARY_COLOR,
                                                    PrimaryStyle.QUATERNARY_COLOR))
                self.plots[-1].update()

            self.root.stl_prev_label_frame_text.set('Slice Preview: %s Sections' % len(cross_sections))
            self.root.stl_prev_window_label_frame.configure(text=self.root.stl_prev_label_frame_text.get())

            self.scroll_window.update()
            self.root.stl_prev_window_label_frame.update()

        def delete_plots(self):
            for plot in reversed(self.plots):
                plot.pack_forget()
                del plot
            del self.plots
            self.plots = list()
            self.root.stl_prev_label_frame_text.set('Slice Preview:')
            self.root.stl_prev_window_label_frame.config(text=self.root.stl_prev_label_frame_text.get())
            self.root.stl_prev_window_label_frame.update()
            self.update()


class DatabaseWindow(EmbeddedWindow):
    SMALL_WEIGHT = 1
    LARGE_WEIGHT = 10

    class CATEGORIES(object):
        PROGRAM_AIRFOILS = 0
        PROGRAM_CAD = 1
        PROJECT_AIRFOILS = 2
        PROJECT_CAD = 3

    def __init__(self, master, root):
        super(DatabaseWindow, self).__init__(master, WindowState.DATABASE, root)

        self.grid_columnconfigure(index=0, weight=self.SMALL_WEIGHT)
        self.grid_columnconfigure(index=1, weight=self.LARGE_WEIGHT)
        self.grid_columnconfigure(index=2, weight=self.SMALL_WEIGHT)
        self.grid_columnconfigure(index=3, weight=self.LARGE_WEIGHT)
        self.grid_columnconfigure(index=4, weight=self.SMALL_WEIGHT)

        self.grid_rowconfigure(index=0, weight=self.SMALL_WEIGHT)
        self.grid_rowconfigure(index=1, weight=self.LARGE_WEIGHT)
        self.grid_rowconfigure(index=2, weight=self.SMALL_WEIGHT)
        self.grid_rowconfigure(index=3, weight=self.LARGE_WEIGHT)
        self.grid_rowconfigure(index=4, weight=self.SMALL_WEIGHT)
        self.grid_propagate(False)

        self.prog_airfoil_list = self.root.project_manager.program_airfoil_dirs
        self.prog_airfoil_var = tk.StringVar(value=self.prog_airfoil_list)

        self.prog_cad_list = self.root.project_manager.program_cad_dirs
        self.prog_cad_var = tk.StringVar(value=self.prog_cad_list)

        self.proj_airfoil_list = self.root.curr_project.database.directory_paths
        self.proj_airfoil_var = tk.StringVar(value=self.proj_airfoil_list)

        self.proj_cad_list = self.root.curr_project.database.directory_paths
        self.proj_cad_var = tk.StringVar(value=self.proj_cad_list)

        # ==============================================================================================================
        # Program Airfoils
        self.prog_airfoil_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                           highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                           highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.prog_airfoil_frame.grid(row=1, column=1, sticky=tk.NSEW)
        self.prog_airfoil_frame.grid_rowconfigure(index=0, weight=1)
        self.prog_airfoil_frame.grid_rowconfigure(index=1, weight=18)
        self.prog_airfoil_frame.grid_rowconfigure(index=2, weight=1)
        self.prog_airfoil_frame.grid_columnconfigure(index=0, weight=1)
        self.prog_airfoil_frame.grid_columnconfigure(index=1, weight=1)

        self.prog_airfoil_label = tk.Label(self.prog_airfoil_frame, text='Program Airfoils:',
                                           bg=PrimaryStyle.TERTIARY_COLOR,
                                           fg=PrimaryStyle.FONT_COLOR,
                                           borderwidth=1, relief=tk.SOLID)
        self.prog_airfoil_label.grid(row=0, column=0, columnspan=2, sticky=tk.NSEW)
        self.prog_airfoils = tk.Listbox(self.prog_airfoil_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        listvariable=self.prog_airfoil_var,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.prog_airfoils.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW)

        self.add_frame_pga = tk.Frame(self.prog_airfoil_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.add_frame_pga.grid(row=2, column=0, sticky=tk.NSEW)
        self.add_frame_pga.grid_rowconfigure(index=0, weight=1)
        self.add_frame_pga.grid_columnconfigure(index=0, weight=1)

        self.add_button_pga = tk.Button(self.add_frame_pga, text='Add',
                                        command=lambda: self._add_selection(self.CATEGORIES.PROGRAM_AIRFOILS),
                                        background=PrimaryStyle.PRIMARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.add_button_pga.grid(row=0, column=0, sticky=tk.NSEW)

        self.del_frame_pga = tk.Frame(self.prog_airfoil_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.del_frame_pga.grid(row=2, column=1, sticky=tk.NSEW)
        self.del_frame_pga.grid_rowconfigure(index=0, weight=1)
        self.del_frame_pga.grid_columnconfigure(index=0, weight=1)

        self.del_button_pga = tk.Button(self.del_frame_pga, text='Delete',
                                        command=lambda: self._delete_selection(self.CATEGORIES.PROGRAM_AIRFOILS),
                                        background=PrimaryStyle.PRIMARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.del_button_pga.grid(row=0, column=0, sticky=tk.NSEW)

        # ==============================================================================================================
        # Program CAD Files
        self.prog_cad_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.prog_cad_frame.grid(row=3, column=1, sticky=tk.NSEW)
        self.prog_cad_frame.grid_rowconfigure(index=0, weight=1)
        self.prog_cad_frame.grid_rowconfigure(index=1, weight=18)
        self.prog_cad_frame.grid_rowconfigure(index=2, weight=1)
        self.prog_cad_frame.grid_columnconfigure(index=0, weight=1)
        self.prog_cad_frame.grid_columnconfigure(index=1, weight=1)

        self.prog_cad_label = tk.Label(self.prog_cad_frame, text='Program Cad Files:',
                                       bg=PrimaryStyle.TERTIARY_COLOR,
                                       fg=PrimaryStyle.FONT_COLOR,
                                       borderwidth=1, relief=tk.SOLID)
        self.prog_cad_label.grid(row=0, column=0, columnspan=2, sticky=tk.NSEW)
        self.prog_cads = tk.Listbox(self.prog_cad_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                    listvariable=self.prog_cad_var,
                                    fg=PrimaryStyle.FONT_COLOR)
        self.prog_cads.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW)

        self.add_frame_pgc = tk.Frame(self.prog_cad_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.add_frame_pgc.grid(row=2, column=0, sticky=tk.NSEW)
        self.add_frame_pgc.grid_rowconfigure(index=0, weight=1)
        self.add_frame_pgc.grid_columnconfigure(index=0, weight=1)

        self.add_button_pgc = tk.Button(self.add_frame_pgc, text='Add',
                                        command=lambda: self._add_selection(self.CATEGORIES.PROGRAM_CAD),
                                        background=PrimaryStyle.PRIMARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.add_button_pgc.grid(row=0, column=0, sticky=tk.NSEW)

        self.del_frame_pgc = tk.Frame(self.prog_cad_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.del_frame_pgc.grid(row=2, column=1, sticky=tk.NSEW)
        self.del_frame_pgc.grid_rowconfigure(index=0, weight=1)
        self.del_frame_pgc.grid_columnconfigure(index=0, weight=1)

        self.del_button_pgc = tk.Button(self.del_frame_pgc, text='Delete',
                                        command=lambda: self._delete_selection(self.CATEGORIES.PROGRAM_CAD),
                                        background=PrimaryStyle.PRIMARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.del_button_pgc.grid(row=0, column=0, sticky=tk.NSEW)

        # ==============================================================================================================
        # Project Airfoils
        self.proj_airfoil_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                           highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                           highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.proj_airfoil_frame.grid(row=1, column=3, sticky=tk.NSEW)
        self.proj_airfoil_frame.grid_rowconfigure(index=0, weight=1)
        self.proj_airfoil_frame.grid_rowconfigure(index=1, weight=18)
        self.proj_airfoil_frame.grid_rowconfigure(index=2, weight=1)
        self.proj_airfoil_frame.grid_columnconfigure(index=0, weight=1)
        self.proj_airfoil_frame.grid_columnconfigure(index=1, weight=1)

        self.proj_airfoil_label = tk.Label(self.proj_airfoil_frame, text='Project Airfoils:',
                                           bg=PrimaryStyle.TERTIARY_COLOR,
                                           fg=PrimaryStyle.FONT_COLOR,
                                           borderwidth=1, relief=tk.SOLID)
        self.proj_airfoil_label.grid(row=0, column=0, columnspan=2, sticky=tk.NSEW)
        self.proj_airfoils = tk.Listbox(self.proj_airfoil_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                        listvariable=self.proj_airfoil_var,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.proj_airfoils.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW)

        self.add_frame_pja = tk.Frame(self.proj_airfoil_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.add_frame_pja.grid(row=2, column=0, sticky=tk.NSEW)
        self.add_frame_pja.grid_rowconfigure(index=0, weight=1)
        self.add_frame_pja.grid_columnconfigure(index=0, weight=1)

        self.add_button_pja = tk.Button(self.add_frame_pja, text='Add',
                                        command=lambda: self._add_selection(self.CATEGORIES.PROJECT_AIRFOILS),
                                        background=PrimaryStyle.PRIMARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.add_button_pja.grid(row=0, column=0, sticky=tk.NSEW)

        self.del_frame_pja = tk.Frame(self.proj_airfoil_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.del_frame_pja.grid(row=2, column=1, sticky=tk.NSEW)
        self.del_frame_pja.grid_rowconfigure(index=0, weight=1)
        self.del_frame_pja.grid_columnconfigure(index=0, weight=1)

        self.del_button_pja = tk.Button(self.del_frame_pja, text='Delete',
                                        command=lambda: self._delete_selection(self.CATEGORIES.PROJECT_AIRFOILS),
                                        background=PrimaryStyle.PRIMARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.del_button_pja.grid(row=0, column=0, sticky=tk.NSEW)

        # ==============================================================================================================
        # Project CAD Files
        self.proj_cad_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.proj_cad_frame.grid(row=3, column=3, sticky=tk.NSEW)
        self.proj_cad_frame.grid_rowconfigure(index=0, weight=1)
        self.proj_cad_frame.grid_rowconfigure(index=1, weight=18)
        self.proj_cad_frame.grid_rowconfigure(index=2, weight=1)
        self.proj_cad_frame.grid_columnconfigure(index=0, weight=1)
        self.proj_cad_frame.grid_columnconfigure(index=1, weight=1)

        self.proj_cad_label = tk.Label(self.proj_cad_frame, text='Project Cad Files:',
                                       bg=PrimaryStyle.TERTIARY_COLOR,
                                       fg=PrimaryStyle.FONT_COLOR,
                                       borderwidth=1, relief=tk.SOLID)
        self.proj_cad_label.grid(row=0, column=0, columnspan=2, sticky=tk.NSEW)
        self.proj_cads = tk.Listbox(self.proj_cad_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                    listvariable=self.proj_cad_var,
                                    fg=PrimaryStyle.FONT_COLOR)
        self.proj_cads.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW)

        self.add_frame_pjc = tk.Frame(self.proj_cad_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.add_frame_pjc.grid(row=2, column=0, sticky=tk.NSEW)
        self.add_frame_pjc.grid_rowconfigure(index=0, weight=1)
        self.add_frame_pjc.grid_columnconfigure(index=0, weight=1)

        self.add_button_pjc = tk.Button(self.add_frame_pjc, text='Add',
                                        command=lambda: self._add_selection(self.CATEGORIES.PROJECT_CAD),
                                        background=PrimaryStyle.PRIMARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.add_button_pjc.grid(row=0, column=0, sticky=tk.NSEW)

        self.del_frame_pjc = tk.Frame(self.proj_cad_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.del_frame_pjc.grid(row=2, column=1, sticky=tk.NSEW)
        self.del_frame_pjc.grid_rowconfigure(index=0, weight=1)
        self.del_frame_pjc.grid_columnconfigure(index=0, weight=1)

        self.del_button_pjc = tk.Button(self.del_frame_pjc, text='Delete',
                                        command=lambda: self._delete_selection(self.CATEGORIES.PROJECT_CAD),
                                        background=PrimaryStyle.PRIMARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.del_button_pjc.grid(row=0, column=0, sticky=tk.NSEW)

    def _add_selection(self, category):
        directory = filedialog.askdirectory()
        if category == self.CATEGORIES.PROGRAM_AIRFOILS:
            self.prog_airfoil_list.append(directory)
            self.prog_airfoil_var.set(self.prog_airfoil_list)
            self.root.project_manager.add_new_airfoil_directory(directory)
            self.root.embedded_windows[WindowState.WING].update_airfoil_options()

        elif category == self.CATEGORIES.PROGRAM_CAD:
            self.prog_cad_list.append(directory)
            self.prog_cad_var.set(self.prog_cad_list)
            self.root.project_manager.add_new_cad_directory(directory)
            self.root.embedded_windows[WindowState.CAD].update_cad_files()

        elif category == self.CATEGORIES.PROJECT_AIRFOILS:
            self.proj_airfoil_list.append(directory)
            self.proj_airfoil_var.set(self.proj_airfoil_list)
            self.root.curr_project.database.add_new_airfoil_directory(directory)
            self.root.embedded_windows[WindowState.WING].update_airfoil_options()

        elif category == self.CATEGORIES.PROJECT_CAD:
            self.proj_cad_list.append(directory)
            self.proj_cad_var.set(self.proj_cad_list)
            self.root.curr_project.database.add_new_cad_directory(directory)
            self.root.embedded_windows[WindowState.CAD].update_cad_files()

    def _delete_selection(self, category):
        if category == self.CATEGORIES.PROGRAM_AIRFOILS:
            selection = self.prog_airfoils.curselection()[0]
            del self.prog_airfoil_list[selection]
            self.prog_airfoil_var.set(self.prog_airfoil_list)

        elif category == self.CATEGORIES.PROGRAM_CAD:
            selection = self.prog_cads.curselection()[0]
            del self.prog_cad_list[selection]
            self.prog_cad_var.set(self.prog_cad_list)

        elif category == self.CATEGORIES.PROJECT_AIRFOILS:
            selection = self.proj_airfoils.curselection()[0]
            del self.proj_airfoil_list[selection]
            self.proj_airfoil_var.set(self.proj_airfoil_list)

        elif category == self.CATEGORIES.PROJECT_CAD:
            selection = self.proj_cads.curselection()[0]
            del self.proj_cad_list[selection]
            self.proj_cad_var.set(self.proj_cad_list)

    def update_from_project(self):
        self.prog_airfoil_list = self.root.project_manager.program_airfoil_dirs
        self.prog_airfoil_var.set(self.prog_airfoil_list)

        self.prog_cad_list = self.root.project_manager.program_cad_dirs
        self.prog_cad_var.set(self.prog_cad_list)

        self.proj_airfoil_list = self.root.curr_project.database.airfoil_directory_paths
        self.proj_airfoil_var.set(self.proj_airfoil_list)

        self.proj_cad_list = self.root.curr_project.database.cad_directory_paths
        self.proj_cad_var.set(self.proj_cad_list)


def get_float(value):
    ret_val = None
    logger = logging.getLogger(__name__)
    try:
        ret_val = float(value)
    except ValueError:
        logger.debug('Could not convert %s to float' % value)
    return ret_val

def get_int(value):
    ret_val = None
    logger = logging.getLogger(__name__)
    try:
        ret_val = int(value)
    except ValueError:
        logger.debug('Could not convert %s to int' % value)
    return ret_val

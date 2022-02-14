import tkinter as tk
import tkinter.ttk as ttk

from .styles import PrimaryStyle


class WindowState(object):
    HOME = 0
    MACHINE_SETUP = 1
    WING = 2
    CAD = 3
    DATABASE = 4
    RANGE = range(0, 5)


class MainWindow(tk.Tk):

    def __init__(self, title, width, height):
        super(MainWindow, self).__init__(baseName=title)
        self.width = width
        self.height = height
        self.title(title)

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

    def switch_embedded_window(self, window_enum):
        self.embedded_windows[window_enum].tkraise()
        print('Changing Window')
        self.update()

    def _create_menu_bar(self):
        file_menu = tk.Menu(self.menu_bar, tearoff=0)
        file_menu.add_command(label="Save Project")
        file_menu.add_command(label="Load Project")
        file_menu.add_command(label="Exit")
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
        self._root = root


class HomeWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(HomeWindow, self).__init__(master, WindowState.HOME, root)

        self.grid_columnconfigure(index=0, weight=5)
        self.grid_columnconfigure(index=1, weight=1)
        self.grid_rowconfigure(index=0, weight=1)

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
        self.left_bot_frame.grid(row=1, column=0, sticky=tk.NSEW,)
        self.left_bot_frame.grid_propagate(False)

        btn_size = PrimaryStyle.HOME_BTN_SIZE
        self.mach_frame = tk.Frame(self.left_top_frame, width=btn_size, height=btn_size)
        self.wing_frame = tk.Frame(self.left_top_frame, width=btn_size, height=btn_size)
        self.body_frame = tk.Frame(self.left_top_frame, width=btn_size, height=btn_size)
        self.db_frame = tk.Frame(self.left_top_frame, width=btn_size, height=btn_size)

        self.machine_button = tk.Button(self.mach_frame, text="CNC Wire \nCutter",
                                        command=lambda: self._root.switch_embedded_window(WindowState.MACHINE_SETUP),
                                        bg=PrimaryStyle.SECONDARY_COLOR,
                                        fg=PrimaryStyle.FONT_COLOR)
        self.wing_button = tk.Button(self.wing_frame, text="Wing \nDesigner",
                                     command=lambda: self._root.switch_embedded_window(WindowState.WING),
                                     bg=PrimaryStyle.SECONDARY_COLOR,
                                     fg=PrimaryStyle.FONT_COLOR)
        self.body_button = tk.Button(self.body_frame, text="CAD \nImport",
                                     command=lambda: self._root.switch_embedded_window(WindowState.CAD),
                                     bg=PrimaryStyle.SECONDARY_COLOR,
                                     fg=PrimaryStyle.FONT_COLOR)
        self.database_button = tk.Button(self.db_frame, text="Edit \nDatabase",
                                         command=lambda: self._root.switch_embedded_window(WindowState.DATABASE),
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

        self.label = ttk.Label(master=self, text='CNC WireCutter')
        self.label.grid()


class WingWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(WingWindow, self).__init__(master, WindowState.WING, root)

        self.label = ttk.Label(master=self, text='Wing')
        self.label.grid()


class CADWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(CADWindow, self).__init__(master, WindowState.CAD, root)

        self.label = ttk.Label(master=self, text='CAD')
        self.label.grid()


class DatabaseWindow(EmbeddedWindow):
    def __init__(self, master, root):
        super(DatabaseWindow, self).__init__(master, WindowState.DATABASE, root)

        self.label = ttk.Label(master=self, text='Database')
        self.label.grid()

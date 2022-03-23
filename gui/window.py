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
        self.grid_rowconfigure(index=1, weight=38)
        self.grid_rowconfigure(index=2, weight=1)
        self.grid_columnconfigure(index=0, weight=9)
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
        self.canvas_frame.grid_columnconfigure(index=0, weight=11)
        self.canvas_frame.grid_columnconfigure(index=1, weight=1)
        self.canvas_frame.grid_propagate(False)

        self.primary_canvas = tk.Canvas(self.canvas_frame,
                                        background=PrimaryStyle.PRIMARY_COLOR, width=65)
        self.primary_canvas.grid(row=0, column=0, sticky=tk.NSEW)
        self.primary_canvas.pack_propagate(False)
        self.primary_canvas.grid_propagate(False)

        self.scroll_window = tk.Frame(self.primary_canvas,
                                      background=PrimaryStyle.PRIMARY_COLOR)

        self.primary_canvas.create_window(0, 0, window=self.scroll_window, anchor=tk.NW)

        self.v_scroll_bar = tk.Scrollbar(self.canvas_frame, orient=tk.VERTICAL, command=self.primary_canvas.yview,
                                         bg=PrimaryStyle.PRIMARY_COLOR)
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
                                       width=65, height=65, text='TEST:%s' % index,
                                       bg=PrimaryStyle.SECONDARY_COLOR,
                                       fg=PrimaryStyle.FONT_COLOR))
        self.items[-1].pack(side=tk.TOP, fill=None, anchor=tk.CENTER)
        if self.curr_select is not None:
            self.items[self.curr_select].button.configure(bg=PrimaryStyle.SECONDARY_COLOR)
            self.items[self.curr_select].button.update()
        self.curr_select = len(self.items) - 1
        self.items[self.curr_select].button.config(background=PrimaryStyle.SELECT_COLOR)
        self.items[self.curr_select].button.update()

    def _delete_selection(self):
        if self.curr_select is not None:
            tmp = self.items[self.curr_select]
            self.items.remove(tmp)
            tmp.destroy()
            del tmp
            self.curr_select = None
            self.scroll_window.update()

        self._recalc_indexes()

    def _recalc_indexes(self):
        for ind in range(len(self.items)):
            self.items[ind].index = ind

    def select(self, index):
        if self.curr_select is not None:
            self.items[self.curr_select].button.configure(bg=PrimaryStyle.SECONDARY_COLOR)
            self.items[self.curr_select].button.update()
        self.curr_select = index
        self.items[self.curr_select].button.configure(bg=PrimaryStyle.SELECT_COLOR)
        self.items[self.curr_select].button.update()
        print('selected button at index: %s' % index)

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
        self.left_bot_frame.grid(row=1, column=0, sticky=tk.NSEW, )
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

        self.grid_rowconfigure(index=0, weight=100)
        self.grid_columnconfigure(index=0, weight=1)
        self.grid_columnconfigure(index=1, weight=8)

        self.primary_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground='#FFFFFF',
                                      highlightthickness=1)
        self.primary_frame.grid(row=0, column=1, sticky=tk.NSEW)

        self.primary_frame.grid_rowconfigure(index=0, weight=1)
        self.primary_frame.grid_columnconfigure(index=0, weight=1)

        self.scroll_frame = ScrollableSelectionMenu(self, root)
        self.scroll_frame.grid(row=0, column=0, sticky=tk.NSEW)

        self._create_top_frame()

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

        self.top_airfoil_frame = self.AirfoilFrame(self.left_top_frame, self, background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.top_airfoil_frame.grid(row=0, column=0, sticky=tk.NSEW)

        self.bot_airfoil_frame = self.AirfoilFrame(self.left_top_frame, self, background=PrimaryStyle.SECONDARY_COLOR,
                                                   highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                   highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.bot_airfoil_frame.grid(row=1, column=0, sticky=tk.NSEW)

        # ==============================================================================================================
        # RIGHT TOP
        self.wing_setting_frame = WingWindow.WingSettingFrame(self.top_frame, root=self,
                                                              background=PrimaryStyle.SECONDARY_COLOR,
                                                              highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                              highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.wing_setting_frame.grid(row=0, column=1, sticky=tk.NSEW)

    def _create_bot_frame(self):
        self.bot_frame = tk.Frame(self.primary_frame, background=PrimaryStyle.SECONDARY_COLOR,
                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
        self.bot_frame.grid(row=1, column=0, sticky=tk.NSEW)

    class AirfoilFrame(tk.Frame):
        def __init__(self, master, root, **kwargs):
            super(WingWindow.AirfoilFrame, self).__init__(master, **kwargs)
            self.root = root

            self.grid_rowconfigure(index=0, weight=4)
            self.grid_rowconfigure(index=1, weight=1)
            self.grid_columnconfigure(index=0, weight=1)

            self.top_frame = tk.Frame(self, background=PrimaryStyle.PRIMARY_COLOR,
                                      highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                      highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.top_frame.grid(row=0, column=0, sticky=tk.NSEW)
            self.top_frame.grid_rowconfigure(index=0, weight=1)
            self.top_frame.grid_columnconfigure(index=0, weight=1)
            self.top_frame.grid_propagate(False)

            self.airfoil_canvas = tk.Canvas(self.top_frame, background=PrimaryStyle.PRIMARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.airfoil_canvas.grid(row=0, column=0, sticky=tk.NSEW)

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
                                                    values=self.get_airfoil_options())
            self.airfoil_option_menu.grid(row=0, column=0, sticky=tk.NSEW, padx=(4, 20), pady=1)

        def get_airfoil_options(self):
            airfoils = list()
            for i in range(1, 100):
                airfoils.append('Airfoil%s' % i)
            return airfoils

    class WingSettingFrame(tk.Frame):
        def __init__(self, master, root, **kwargs):
            super(WingWindow.WingSettingFrame, self).__init__(master, **kwargs)
            self.root = root

            self.grid(row=0, column=1, sticky=tk.NSEW)
            self.grid_rowconfigure(index=0, weight=4)
            self.grid_rowconfigure(index=1, weight=1)
            self.grid_rowconfigure(index=2, weight=1)
            self.grid_columnconfigure(index=0, weight=1)

            self.wing_field_frame = tk.Frame(self, background=PrimaryStyle.SECONDARY_COLOR,
                                             highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                             highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.wing_field_frame.grid(row=0, column=0, sticky=tk.NSEW)
            self.wing_field_frame.grid_rowconfigure(index=0, weight=5)
            self.wing_field_frame.grid_rowconfigure(index=1, weight=5)
            self.wing_field_frame.grid_rowconfigure(index=2, weight=5)
            self.wing_field_frame.grid_rowconfigure(index=3, weight=1)
            self.wing_field_frame.grid_rowconfigure(index=4, weight=1)
            self.wing_field_frame.grid_columnconfigure(index=0, weight=1)

            self.name_label_frame = tk.LabelFrame(self.wing_field_frame, text='Name:',
                                                  background=PrimaryStyle.SECONDARY_COLOR,
                                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                  fg=PrimaryStyle.FONT_COLOR, width=5)
            self.name_label_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING,
                                       pady=PrimaryStyle.GENERAL_PADDING / 2)

            self.span_label_frame = tk.LabelFrame(self.wing_field_frame, text='Span (mm):',
                                                  background=PrimaryStyle.SECONDARY_COLOR,
                                                  highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                  highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                  fg=PrimaryStyle.FONT_COLOR, width=10)
            self.span_label_frame.grid(row=1, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING,
                                       pady=PrimaryStyle.GENERAL_PADDING / 2)

            self.washout_label_frame = tk.LabelFrame(self.wing_field_frame, text='Washout (deg):',
                                                     background=PrimaryStyle.SECONDARY_COLOR,
                                                     highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                     highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                     fg=PrimaryStyle.FONT_COLOR, width=14)
            self.washout_label_frame.grid(row=2, column=0, sticky=tk.NSEW,
                                          padx=PrimaryStyle.GENERAL_PADDING,
                                          pady=PrimaryStyle.GENERAL_PADDING / 2)

            self.gen_left_right_frame = tk.Frame(self.wing_field_frame,
                                                 background=PrimaryStyle.SECONDARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.gen_left_right_frame.grid(row=3, column=0, sticky=tk.NSEW)
            self.gen_left_right_frame.grid_rowconfigure(index=0, weight=1)
            self.gen_left_right_frame.grid_columnconfigure(index=0, weight=1)
            self.gen_left_right_frame.pack_propagate(False)

            self.gen_left_right = tk.Checkbutton(self.gen_left_right_frame, text='Generate Left/Right G-Code',
                                                 background=PrimaryStyle.SECONDARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                 fg=PrimaryStyle.FONT_COLOR, width=26)
            self.gen_left_right.pack(anchor=tk.NW)

            self.align_led_edge_frame = tk.Frame(self.wing_field_frame,
                                                 background=PrimaryStyle.SECONDARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.align_led_edge_frame.grid(row=4, column=0, sticky=tk.NSEW)
            self.align_led_edge_frame.grid_rowconfigure(index=0, weight=1)
            self.align_led_edge_frame.grid_columnconfigure(index=0, weight=1)
            self.align_led_edge_frame.pack_propagate(False)

            self.align_led_edge = tk.Checkbutton(self.align_led_edge_frame, text='Align leading edge to wire',
                                                 background=PrimaryStyle.SECONDARY_COLOR,
                                                 highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                 highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                 fg=PrimaryStyle.FONT_COLOR, width=26)
            self.align_led_edge.pack(anchor=tk.NW)

            self.slice_btn_frame = tk.Frame(self, background=PrimaryStyle.SECONDARY_COLOR,
                                            highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                            highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS)
            self.slice_btn_frame.grid(row=1, column=0, sticky=tk.NSEW)

            self.sel_cnc_machine_frame = tk.LabelFrame(self, background=PrimaryStyle.SECONDARY_COLOR,
                                                       highlightbackground=PrimaryStyle.HL_BACKGROUND_COL,
                                                       highlightthickness=PrimaryStyle.HL_BACKGROUND_THICKNESS,
                                                       fg=PrimaryStyle.FONT_COLOR,
                                                       text='Select CNC Machine:')
            self.sel_cnc_machine_frame.grid(row=2, column=0, sticky=tk.NSEW, padx=PrimaryStyle.GENERAL_PADDING,
                                            pady=PrimaryStyle.GENERAL_PADDING)



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

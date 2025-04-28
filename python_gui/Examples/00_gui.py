import tkinter as tk
from tkinter import scrolledtext
from tkinter import font
import time

DEBUG = True


class SaraGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Sara Development User Interface - BODY")

        # Set up the window close protocol
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Create the main frame
        self.frame = tk.Frame(self.root)
        self.frame.pack(padx=20, pady=20)

        # --------------------------------------------------------------------------------------
        # Left arm
        # --------------------------------------------------------------------------------------
        self.button_debug1 = tk.Button(self.frame, height=1, width=10, text="Debug 1")
        self.button_debug1.grid(row=0, column=9, sticky="w")

        self.button_LA_White = tk.Button(
            self.frame,
            height=1,
            width=10,
            text="Left White",
        )
        self.button_LA_White.grid(row=0, column=0, sticky="w")

        self.button_LA_Red = tk.Button(
            self.frame,
            height=1,
            width=10,
            text="Left Red",
        )
        self.button_LA_Red.grid(row=1, column=0, sticky="w")

        self.button_LA_Green = tk.Button(
            self.frame,
            height=1,
            width=10,
            text="Left Green",
        )
        self.button_LA_Green.grid(row=2, column=0, sticky="w")

        self.button_LA_Blue = tk.Button(
            self.frame,
            height=1,
            width=10,
            text="Left Blue",
        )
        self.button_LA_Blue.grid(row=3, column=0, sticky="w")

        self.button_LA_Off = tk.Button(
            self.frame,
            height=1,
            width=10,
            text="Left Off",
        )
        self.button_LA_Off.grid(row=4, column=0, sticky="w")

        self.button_LA_Blue_Blink = tk.Button(
            self.frame,
            height=1,
            width=10,
            text="Left Blue Blink",
        )
        self.button_LA_Blue_Blink.grid(row=5, column=0, sticky="w")

        # --------------------------------------------------------------------------------------
        # streaming data area
        # --------------------------------------------------------------------------------------
        self.stream_area = scrolledtext.ScrolledText(
            self.frame, width=100, height=20, wrap="word"
        )
        self.stream_area.grid(row=10, column=0, columnspan=10, sticky="w")

    def on_closing(self):
        time.sleep(1)
        self.root.destroy()  # This will close the window

    def run(self):
        self.root.mainloop()


def main():
    gui = SaraGUI()
    gui.run()


if __name__ == "__main__":
    main()

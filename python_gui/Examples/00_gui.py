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
        # General settings
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count = 0
        button_width = 15

        self.button_version_head = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Version Head",
        )
        self.button_version_head.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_version_body = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Version Body",
        )
        self.button_version_body.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Left arm
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count = 1

        self.button_LA_White = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left White",
        )
        self.button_LA_White.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Red",
        )
        self.button_LA_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Green = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Green",
        )
        self.button_LA_Green.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Blue = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Blue",
        )
        self.button_LA_Blue.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Blue_Blink = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Blue Blink",
        )
        self.button_LA_Blue_Blink.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Off",
        )
        self.button_LA_Off.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Right arm
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count = 2

        self.button_RA_White = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right White",
        )
        self.button_RA_White.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Red",
        )
        self.button_RA_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Green = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Green",
        )
        self.button_RA_Green.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Blue = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Blue",
        )
        self.button_RA_Blue.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Green_Blink = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Green Blink",
        )
        self.button_RA_Green_Blink.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Off",
        )
        self.button_RA_Off.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Base
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count += 1

        self.button_BASE_White = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base White",
        )
        self.button_BASE_White.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Red",
        )
        self.button_BASE_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Green = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Green",
        )
        self.button_BASE_Green.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Blue = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Blue",
        )
        self.button_BASE_Blue.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Blink_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Blink Red",
        )
        self.button_BASE_Blink_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Off",
        )
        self.button_BASE_Off.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Head
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count += 1

        self.button_HEAD_Left_White = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Left White",
        )
        self.button_HEAD_Left_White.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_HEAD_Left_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right Red",
        )
        self.button_HEAD_Left_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_HEAD_Left_Green = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right Green",
        )
        self.button_HEAD_Left_Green.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_HEAD_Left_Blue = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right Blue",
        )
        self.button_HEAD_Left_Blue.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_HEAD_Left_White_Blink = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right White Blink",
        )
        self.button_HEAD_Left_White_Blink.grid(
            row=row_count, column=col_count, sticky="w"
        )

        row_count += 1

        self.button_HEAD_Left_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right Off",
        )
        self.button_HEAD_Left_Off.grid(row=row_count, column=col_count, sticky="w")
        # --------------------------------------------------------------------------------------
        # Head Right
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count = 4

        self.button_HEAD_Right_White = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right White",
        )
        self.button_HEAD_Right_White.grid(row=row_count, column=col_count, sticky="w")

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

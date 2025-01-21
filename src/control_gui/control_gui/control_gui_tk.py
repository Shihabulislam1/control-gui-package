import tkinter as tk


def main():
    root = tk.Tk()
    root.title("Hello GUI")

    label = tk.Label(root, text="Hello")
    label.pack(padx=20, pady=20)

    root.mainloop()

if __name__ == "__main__":
    main()
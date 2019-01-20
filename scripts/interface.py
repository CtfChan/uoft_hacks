import tkinter as tk
 
from tkinter import messagebox

def showModal(s):
  top = tk.Toplevel(master=root)
  top.title('Message')
  top.geometry("1000x400")
  top.resizable(False, False)
  top.transient(root)
  top.grab_set()

  lbl = tk.Label(top, text = s)
  lbl.configure(font=('Helvetica', 50))
  lbl.pack()
  tk.Button(top, text=" OK ",font=('Helvetica', 50), command=top.destroy).pack(side=tk.BOTTOM)

root = tk.Tk()
 
root.title("Welcome to the Health Assistant")
 
selected = tk.IntVar()

root.geometry('1300x900')

lbl = tk.Label(root, text="Which medication have you taken?", font=("Arial Bold", 50))
 
lbl.grid(column=0, row=0)
 
myFont = ('Arial', 32)
rad1 = tk.Radiobutton(root,text='Morning Medication', value=1, variable=selected,font=("Arial Bold", 50))
 
rad2 = tk.Radiobutton(root,text='Midday Medication ', value=2, variable=selected,font=("Arial Bold", 50))
 
rad3 = tk.Radiobutton(root,text='Evening Medication', value=3, variable=selected,font=("Arial Bold", 50))


 
def clicked():
    if selected.get() == 1:
        message = "\nGreat! Remember to take your\nMidday Medication :)"    
        # msg = tk.Message(window, text = message)
        # msg.config(bg='lightgreen', font=('times', 50, 'italic'))
        #msg.pack()

    elif selected.get() == 2:
        message = "\nGreat! Remember to take your\nEvening Medication :)"
    elif selected.get() == 3:
        message = "Great! You've take your all your\nmedication for the day.\nHave a good night :)"

    
    showModal(message)
    print(selected.get())
 
btn = tk.Button(root, text="Click Me",command=clicked,font=("Arial Bold", 50))
 
rad1.grid(column=0, row=1)
 
rad2.grid(column=0, row=2)
 
rad3.grid(column=0, row=3)
 
btn.grid(column=0, row=4)

col_count, row_count = root.grid_size()

for col in range(col_count):
    root.grid_columnconfigure(col, minsize=10)

for row in range(row_count):
    root.grid_rowconfigure(row, minsize=150)

root.mainloop()

#!/usr/bin/env python
# coding: utf-8

import tkinter as tk
import datetime
import math
import copy
import heapq

class Gui:
    
    def __init__(self):
        
        #initialization of the board
        
        self.root = tk.Tk()
        self.root.title("Shortest path finder")
        self.root.attributes('-topmost', 'true')

        self.start_window=None
        
        self.w=650
        self.h=684
        self.ws = self.root.winfo_screenwidth()
        self.hs = self.root.winfo_screenheight()
        self.x = (self.ws/2) - (self.w/2)
        self.y = (self.hs/2) - (self.h/2)
        
        self.root.geometry(f"{self.w}x{self.h}+{int(self.x)}+{int(self.y)}")
        
        #self.vis_df is the matrix of boxes displayed on the screen
        #self.hid_df is the numeric matrix not printed on the screen that is used to compute Lee's algorithm
        
        self.vis_df=[[0 for x in range(25)] for row in range(25)]
        self.hid_df=[[0 for x in range(25)] for row in range(25)]
        
        #initialization of the bottom part of the window
        
        self.bottomframe=tk.LabelFrame(self.root, pady=2, relief="flat")
        self.bottomframe.grid(row=26, columnspan=25)
        
        self.reset_but=tk.Button(self.bottomframe, text="Reset button", command=self.reset)
        self.reset_but.grid(row=0, column=0, padx=50)
        
        self.start_but=tk.Button(self.bottomframe, text="Start", command=start)
        self.start_but.grid(row=0, column=1, padx=50)
        
        self.time_but=tk.Label(self.bottomframe, font = ('calibri', 12))
        self.time_but.grid(row=0, column=2, padx=50)
        self.start_time=datetime.datetime.now()
        self.timefunc()
        
        #initialization of the boxes constituting the grid
        
        self.i=tk.PhotoImage()
        
        for x in range(25):
            for y in range(25):

                self.vis_df[x][y]=tk.Label(self.root, text=" ", image=self.i, relief="solid", width=20, height=20, compound="center", bg="white")
                self.vis_df[x][y].grid(row=x, column=y)
        
        self.vis_df[1][1].config(bg="light grey", text="S", image=self.i)
        self.vis_df[23][23].config(bg="firebrick1", text="E", image=self.i)
        
        #initialization of the graph
        
        self.graph=self.graph_init(25)
        
        #various board attributes
        
        self.start_x=1
        self.start_y=1

        self.end_x=23
        self.end_y=23
        
        self.blue_code=150
        self.green_code=0
        self.red_code=0

        #activates the start window

        self.choice=tk.IntVar()
        self.choice.set(0)
        self.starting_window()
        
    def mainloop(self):
        
        self.root.mainloop()
            
    def reset(self):
        
        #resets the board to the initial state
        
        self.start_time=datetime.datetime.now()
        
        self.hid_df=[[0 for x in range(25)] for row in range(25)]
        
        for x in range(25):
            for y in range(25):

                self.vis_df[x][y].config(bg="white")
        
        self.vis_df[1][1].config(bg="light grey")
        self.vis_df[23][23].config(bg="firebrick1")
        
        self.blue_code=150
        self.green_code=0
        self.red_code=0

        #resets the graph to the starting condition

        self.graph=self.graph_init(25)
        
        #re-enables the start button
        
        self.start_but.config(state="active")

        self.starting_window()
    
        
    def ending_window(self, outcome, path_len):

        #this method creates a window when the algorithm's execution has ended
        
        top=tk.Toplevel()
        top.title("")
        top.geometry(f"{305}x{79}+{int((self.ws/2) - (200/2))}+{int((self.hs/2) - (79/2))}")
        top.overrideredirect(True)
        top.lift(self.root)
        popframe=tk.LabelFrame(top)
        popframe.place(relx=0.5, rely=0.06, anchor="n")

        if outcome==1 and path_len==None:
            
            tk.Label(popframe, text=f"A solution has been found and it is {self.hid_df[23][23]} blocks long").pack(padx=10, pady=5)

        elif outcome==1 and path_len!=None:

            tk.Label(popframe, text=f"A solution has been found and it is {path_len} blocks long").pack(padx=10, pady=5)
            
        else:
            
            tk.Label(popframe, text="There is no path available to the end point").pack(padx=30, pady=5)
        
        tk.Button(popframe, text="Close this window", command=top.destroy).pack(pady=5)

    def starting_window(self):

        #starting window that allow the user to choose the algorithm
        
        self.start_window=tk.Toplevel(self.root)
        self.start_window.title("")
        self.start_window.geometry(f"{180}x{165}+{int((self.ws/2) - (200/2))}+{int((self.hs/2) - (79/2))}")
        self.start_window.overrideredirect(True)
        self.start_window.lift(self.root)
        my_frame=tk.LabelFrame(self.start_window)
        my_frame.place(relx=0.5, rely=0.06, anchor="n")

        tk.Radiobutton(my_frame, text="Lee's algorithm", variable=self.choice, value=0).pack(padx=10, pady=5)
        tk.Radiobutton(my_frame, text="Dijkstra's algorithm", variable=self.choice, value=1).pack(padx=10, pady=5)
        tk.Radiobutton(my_frame, text="A-star algorithm", variable=self.choice, value=2).pack(padx=10, pady=5)
        tk.Button(my_frame, text="Close this window", command=self.start_bindings).pack(padx=10, pady=5)
    
    def start_bindings(self):

        #closes the starting window

        self.start_window.destroy()

        #initialize the mouse bindings

        self.root.bind("<B1-Motion>", self.drag) 
        self.root.bind("<ButtonPress-1>", self.click)

      
    def timefunc(self):
        
        #code that manages the clock
        
        current_time=datetime.datetime.now()
        seconds=(current_time-self.start_time).seconds
        minutes=int(seconds/60)
            
        if seconds-60*minutes<10:
            seconds="0"+str(seconds-60*minutes)

        elif seconds>=60:
            seconds=seconds-60*minutes
                
        self.time_but.config(text = str(minutes)+":"+str(seconds))
        self.time_but.after(1000, self.timefunc)
    
    def graph_init(self, side):

        #loop that creates the graph
        
        graph={}
    
        for i in range(side**2):
            if i==0:
                graph[i]={(1):1,(side):1}
            elif i==(side-1):
                graph[i]={(i-1):1,(i+side):1}
            elif i==side**2-side:
                graph[i]={(i+1):1,(i-side):1}
            elif i==side**2-1:
                graph[i]={(i-1):1,(i-side):1}
            elif i<side:
                graph[i]={(i-1):1,(i+1):1,(i+side):1}
            elif i>(side**2-side-1):
                graph[i]={(i-1):1,(i+1):1,(i-side):1}
            elif i%side==0:
                graph[i]={(i-side):1,(i+side):1,(i+1):1}
            elif (i+1)%side==0:
                graph[i]={(i-side):1,(i+side):1,(i-1):1}
            else:
                graph[i]={(i-side):1,(i+side):1,(i-1):1,(i+1):1}

        return graph

    def click(self, l):
        
        #takes the input when clicking the first time on the board and fills the clicked boxes
        
        click_y = int((self.root.winfo_pointerx() - self.root.winfo_rootx())/26)
        click_x = int((self.root.winfo_pointery() - self.root.winfo_rooty())/26)

        if 0<=click_y<25 and 0<=click_x<25 and self.vis_df[click_x][click_y]["bg"]=="white":
            
            number=(click_x)*25+click_y
            self.vis_df[click_x][click_y].config(bg="black")
            self.hid_df[click_x][click_y]="X"

            for i in self.graph[number]:
                
                self.graph[i].pop(number)
    
    def drag(self, l):
        
        #takes the input while dragging the mouse of the board and fills the clicked boxes
        
        drag_y = int((self.root.winfo_pointerx() - self.root.winfo_rootx())/26)
        drag_x = int((self.root.winfo_pointery() - self.root.winfo_rooty())/26)
        
        if 0<=drag_y<25 and 0<=drag_x<25 and self.vis_df[drag_x][drag_y]["bg"]=="white":

            number=(drag_x)*25+drag_y
            self.vis_df[drag_x][drag_y].config(bg="black")
            self.hid_df[drag_x][drag_y]="X"
            
            for i in self.graph[number]:
                
                self.graph[i].pop(number)


def start():
    
    #this function disables the two buttons and launches the chosen algorithm
    
    myboard.reset_but.config(state="disabled")

    myboard.start_but.config(state="disabled")

    #removes the mouse bindings

    myboard.root.unbind("<B1-Motion>") 
    myboard.root.unbind("<ButtonPress-1>")

    #algorithm initialization based on the user's choice
    
    if myboard.choice.get()==0:
        lee()
    elif myboard.choice.get()==1:
        dijkstra(myboard.graph)
    else:
        astar(myboard.graph)

#this part of code executes Lee's algorithm

def lee():
    
    #the starting box is associates with number 1
    
    myboard.hid_df[myboard.start_x][myboard.start_y]=1
    
    #the recursive function is launched
    
    if lee_solver([[myboard.start_x, myboard.start_y]], 2):
        
        myboard.ending_window(1, None)

    else:
        
        myboard.ending_window(0, None)
        
    #after the execution of the algorithm the reset button is clickable again
    
    myboard.reset_but.config(state="active")
    
    
def lee_solver(mylist, count):

    #the moves that can be made at every step

    move_x=[0,0,1,-1]
    move_y=[1,-1,0,0]
        
    newlist=[]

    counter=0

    for i in range(len(mylist)):

        for l in range(4):

            new_x=mylist[i][0]+move_x[l]
            new_y=mylist[i][1]+move_y[l]

            if new_x==myboard.end_x and new_y==myboard.end_y:
                myboard.hid_df[new_x][new_y]=count
                lee_traceback(new_x, new_y, move_x, move_y)
                return True

            elif lee_validcell(new_x, new_y):
                counter+=1
                myboard.hid_df[new_x][new_y]=count
                myboard.root.after(10, lee_fill_box(new_x, new_y))
                newlist.append([new_x, new_y])
    
    lee_color_updater()

    if counter==0:
        return False

    count += 1

    if lee_solver(newlist, count):
        return True
    
def lee_traceback(x, y, move_x, move_y):

        
    while x!=myboard.start_x or y!=myboard.start_y:
    
        myboard.root.after(20, myboard.vis_df[x][y].config(bg="firebrick1"))
        myboard.root.update()

        for i in range(4):

            new_x=x+move_x[i]
            new_y=y+move_y[i]

            if 0<=new_x<25 and 0<=new_y<25 and 0<=x<25 and 0<=y<25 and myboard.hid_df[new_x][new_y]==myboard.hid_df[x][y]-1:

                x=new_x
                y=new_y

                break

    myboard.vis_df[myboard.start_x][myboard.start_y].config(bg="firebrick1")
    
def lee_validcell(x, y):
        
    #this method checks whether the adjacent box can be filled or not. It must not be a wall or be located outside the board
    
    if x>=0 and y>=0 and x<25 and y<25 and myboard.hid_df[x][y]==0:
        return True
    else:
        return False

def lee_color_updater():
        
    #this method is called at every new iteration of Lee's algorithm and it updates the colors used to fill the boxes

    if myboard.blue_code<246:
        myboard.blue_code+=5
    elif myboard.green_code<246:
        myboard.green_code+=5
    elif myboard.red_code<150:
        myboard.red_code+=5
    
def lee_fill_box(new_x, new_y):
        
    #this method fills the boxes during the execution of Lee's algorithm
    
    myboard.vis_df[new_x][new_y].config(bg=f"#{myboard.red_code:02x}{myboard.green_code:02x}{myboard.blue_code:02x}")
    myboard.root.update()
    
#this part of code executes Dijkstra's algorithm

def dijkstra(graph):

    global myboard

    shortest_dist={}
    base_dist=0
    steps={}
    path=[]
    choose={}

    source=26
    target=598
    
    #at the beginning every node has distance set to infinite except the starting one

    for nodes in graph:
        
        shortest_dist[nodes]=math.inf
    
    shortest_dist[source]=0
    
    current_node=source

    pq = [(0, source)]
        
    while len(pq)>0:

        base_dist, current_node = heapq.heappop(pq)
    
    #after having defined the current node this loop interacts with every neighbour of that node updating them if necessary
    
        for next_node in graph[current_node]:

            choose[current_node]=base_dist
            
            if base_dist+graph[current_node][next_node]<shortest_dist[next_node]:
                
                shortest_dist[next_node]=base_dist+graph[current_node][next_node]
                
                row=int(next_node)//25
                col=int(next_node)%25
                myboard.root.after(10, myboard.vis_df[row][col].config(bg="green4"))
                myboard.root.update()
                heapq.heappush(pq, (base_dist+graph[current_node][next_node], next_node))
                
                steps[next_node]=current_node
                
                #if the target has been reached close the algorithm with this code
                
                if next_node==target:
                    
                    path.append(target)
                    key=steps[target]

                    while steps[target]!=source:
                        
                        try:
                            path.append(key)
                            key=steps[key]
                        except:
                            break
                    myboard.reset_but.config(state="active")

                    return ending(path)
    
        #in case this iteration of neighbours didn't find the target start the next one removing the previous node
        
        #this line of code determines the next node

        row=current_node//25
        col=current_node%25
        myboard.vis_df[row][col].config(bg="SpringGreen3")
        myboard.root.update()

    #if all the connections have been explored and the target was not found
    
    myboard.ending_window(0, None)

    myboard.reset_but.config(state="active") 

#this part of code executes the A star algorithm

def astar_calc_h(side, start, end_node):
    
    start_x=start//side
    start_y=start%side
    end_x=end_node//side
    end_y=end_node%side
    return (abs(start_x-end_x)**2+abs(start_y-end_y)**2)**0.5

#selects the node in the open list with lowest f value
#if there are more nodes with the lowest f value, the choice is made using the h value

def astar_min_value(nodes, open_list):
    
    my_dict={}
    
    min_value=min(open_list.items(), key=lambda x: x[1])

    for key, value in open_list.items():
    
        if value==min_value[1]:
            my_dict[key]=nodes[key].h
    
    if len(my_dict)==1:
        return min_value[0]
    else:
        return min(my_dict.items(), key=lambda x: x[1])[0]

#this function draws the shortest path from start to end node moving backward

def astar_traceback(nodes, start_node, end_node):
    
    x=end_node
    path=[end_node]
    while nodes[x].parent!=start_node:
        x=nodes[x].parent
        path.append(x)
    path.append(start_node)
    return path

#this is the class for every node of the graph used in the A star algorithm  

class node:
    
    def __init__(self, pos, g, h):
        
        self.position=pos
        self.parent={}
        self.g=g
        self.h=h
        self.f=self.g+self.h
        
    def insert_parent(self, parent):
        
        self.parent=parent

#this part of code executes the A star algorithm

def astar(graph):

    side=25

    start_node=26
    end_node=598

    nodes={}
    
    nodes[start_node]=node(start_node, 0, astar_calc_h(side, start_node, end_node))

    open_list={}
    closed_list=[]

    open_list[start_node]=nodes[start_node].f

    while len(open_list)>0:
        
        #this line of code determines the node in the open list with the lowest f or g value

        current_node = astar_min_value(nodes, open_list)

        #these lines of code change the color of the grid's boxes 

        if current_node!=start_node:
            row=current_node//25
            col=current_node%25
            myboard.vis_df[row][col].config(bg="yellow2")
            myboard.root.update()
        
        closed_list.append(current_node)
        open_list.pop(current_node)

        if current_node==end_node:
            path=astar_traceback(nodes, start_node, end_node)
            ending(path)
            myboard.reset_but.config(state="active") 
            return
        
        #this loop examines every neighbour of the current node

        for key, value in graph[current_node].items():
            
            #if the neighbour was already selected as current node skip to the next iteration

            if key in closed_list:
                continue
            
            #the neighbour node is added to the open list if it is not already present

            if key not in open_list:
                
                nodes[key]=node(key, nodes[current_node].g+value, astar_calc_h(side, key, end_node))
                open_list[key]=nodes[key].f
                nodes[key].insert_parent(current_node)
                row=key//25
                col=key%25
                myboard.root.after(10, myboard.vis_df[row][col].config(bg="orange"))
                myboard.root.update()

            #if the neighbour node is already in the open list but a new path that results in a lowest f is found
            #change the f value of the neighbour to this new value and change his parent node

            elif key in open_list and nodes[current_node].g+value<nodes[key].g:
                
                print("clause used")
                nodes[key].g=nodes[current_node].g+value
                open_list[key]=nodes[key].f
                nodes[key].insert_parent(current_node)

    #if the target was not found after exploring all the available nodes

    myboard.ending_window(0, None)

    myboard.reset_but.config(state="active") 
    
def ending(path):
        
    #this function draws the shortest path

    for i in path:
        myboard.root.after(20, myboard.vis_df[i//25][i%25].config(bg="firebrick1"))
        myboard.root.update()
    return myboard.ending_window(1, len(path))


myboard=Gui()

myboard.mainloop()
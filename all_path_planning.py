from tkinter import *
from A_star import doAStarPathPlanning
from Dijkstra import doDijkstra
from BFS import doBFS
from DFS import doDFS
from RRT import doRRT
from RRT_Star import doRRTStar

# Call tkinter Window and display all options

if __name__ == "__main__":
    # create a tkinter window

    root = Tk()

    # Set tkinter menu Title

    root.title('Planning Menu')

    # Set tkinter window dimensions

    root.geometry('300x300')

    # DFS Path Planning Button

    btn_dfs = Button(root, text='Solve by DFS Path Planning', bd='10',
                     command=doDFS)

    # BFS Path Planning Button

    btn_bfs = Button(root, text='Solve by BFS Path Planning', bd='10',
                     command=doBFS)

    # A star Path Planning Button

    btn_a_star = Button(root, text='Solve by A * Path Planning', bd='10',
                        command=doAStarPathPlanning)

    # Dijkstra Path Planning Button

    btn_dijkstra = Button(root, text='Solve by Dijkstra Path Planning', bd='10',
                          command=doDijkstra)

    # RRT Path Planning Button

    btn_RRT = Button(root, text='Solve by RRT Path Planning', bd='10',
                     command=doRRT)

    # RRT * Path Planning Button

    btn_RRT_Star = Button(root, text='Solve by RRT Star Path Planning', bd='10',
                          command=doRRTStar)

    # Exit Button

    exit_button = Button(root, text='Exit!', bd='10',
                         command=root.destroy)

    # Set button positions

    btn_dfs.pack(side='top')
    btn_bfs.pack(side='top')
    btn_a_star.pack(side='top')
    btn_dijkstra.pack(side='top')
    btn_RRT.pack(side='top')
    btn_RRT_Star.pack(side='top')

    # Exit Button to be placed differently

    exit_button.pack(side='bottom')

    root.mainloop()

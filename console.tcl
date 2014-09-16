# Tcl commands to run in the console before qrouter is initialized
 
slave alias qrouter::consoledown wm withdraw .
slave alias qrouter::consoleup wm deiconify .
slave alias qrouter::consoleontop raise .

wm protocol . WM_DELETE_WINDOW {tkcon slave slave qrouter::lowerconsole}

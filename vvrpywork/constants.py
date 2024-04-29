from pyglet.window import key
from pyglet.window import mouse

class Key:
    # Modifier mask constants
    MOD_SHIFT       = key.MOD_SHIFT
    MOD_CTRL        = key.MOD_CTRL
    MOD_ALT         = key.MOD_ALT
    MOD_CAPSLOCK    = key.MOD_CAPSLOCK
    MOD_NUMLOCK     = key.MOD_NUMLOCK
    MOD_WINDOWS     = key.MOD_WINDOWS
    MOD_COMMAND     = key.MOD_COMMAND
    MOD_OPTION      = key.MOD_OPTION
    MOD_SCROLLLOCK  = key.MOD_SCROLLLOCK
    MOD_FUNCTION    = key.MOD_FUNCTION

    #: Accelerator modifier.  On Windows and Linux, this is ``MOD_CTRL``, on
    #: Mac OS X it's ``MOD_COMMAND``.
    MOD_ACCEL = key.MOD_ACCEL


    # Key symbol constants

    # ASCII commands
    BACKSPACE     = key.BACKSPACE
    TAB           = key.TAB
    LINEFEED      = key.LINEFEED
    CLEAR         = key.CLEAR
    RETURN        = key.RETURN
    ENTER         = key.ENTER      # synonym
    PAUSE         = key.PAUSE
    SCROLLLOCK    = key.SCROLLLOCK
    SYSREQ        = key.SYSREQ
    ESCAPE        = key.ESCAPE
    SPACE         = key.SPACE

    # Cursor control and motion
    HOME          = key.HOME
    LEFT          = key.LEFT
    UP            = key.UP
    RIGHT         = key.RIGHT
    DOWN          = key.DOWN
    PAGEUP        = key.PAGEUP
    PAGEDOWN      = key.PAGEDOWN
    END           = key.END
    BEGIN         = key.BEGIN

    # Misc functions
    DELETE        = key.DELETE
    SELECT        = key.SELECT
    PRINT         = key.PRINT
    EXECUTE       = key.EXECUTE
    INSERT        = key.INSERT
    UNDO          = key.UNDO
    REDO          = key.REDO
    MENU          = key.MENU
    FIND          = key.FIND
    CANCEL        = key.CANCEL
    HELP          = key.HELP
    BREAK         = key.BREAK
    MODESWITCH    = key.MODESWITCH
    SCRIPTSWITCH  = key.SCRIPTSWITCH
    FUNCTION      = key.FUNCTION

    # Text motion constants: these are allowed to clash with key constants
    MOTION_UP                = key.MOTION_UP
    MOTION_RIGHT             = key.MOTION_RIGHT
    MOTION_DOWN              = key.MOTION_DOWN
    MOTION_LEFT              = key.MOTION_LEFT
    MOTION_NEXT_WORD         = key.MOTION_NEXT_WORD
    MOTION_PREVIOUS_WORD     = key.MOTION_PREVIOUS_WORD
    MOTION_BEGINNING_OF_LINE = key.MOTION_BEGINNING_OF_LINE
    MOTION_END_OF_LINE       = key.MOTION_END_OF_LINE
    MOTION_NEXT_PAGE         = key.MOTION_NEXT_PAGE
    MOTION_PREVIOUS_PAGE     = key.MOTION_PREVIOUS_PAGE
    MOTION_BEGINNING_OF_FILE = key.MOTION_BEGINNING_OF_FILE
    MOTION_END_OF_FILE       = key.MOTION_END_OF_FILE
    MOTION_BACKSPACE         = key.MOTION_BACKSPACE
    MOTION_DELETE            = key.MOTION_DELETE
    MOTION_COPY              = key.MOTION_COPY
    MOTION_PASTE             = key.MOTION_PASTE

    # Number pad
    NUMLOCK       = key.NUMLOCK
    NUM_SPACE     = key.NUM_SPACE
    NUM_TAB       = key.NUM_TAB
    NUM_ENTER     = key.NUM_ENTER
    NUM_F1        = key.NUM_F1
    NUM_F2        = key.NUM_F2
    NUM_F3        = key.NUM_F3
    NUM_F4        = key.NUM_F4
    NUM_HOME      = key.NUM_HOME
    NUM_LEFT      = key.NUM_LEFT
    NUM_UP        = key.NUM_UP
    NUM_RIGHT     = key.NUM_RIGHT
    NUM_DOWN      = key.NUM_DOWN
    NUM_PRIOR     = key.NUM_PRIOR
    NUM_PAGE_UP   = key.NUM_PAGE_UP
    NUM_NEXT      = key.NUM_NEXT
    NUM_PAGE_DOWN = key.NUM_PAGE_DOWN
    NUM_END       = key.NUM_END
    NUM_BEGIN     = key.NUM_BEGIN
    NUM_INSERT    = key.NUM_INSERT
    NUM_DELETE    = key.NUM_DELETE
    NUM_EQUAL     = key.NUM_EQUAL
    NUM_MULTIPLY  = key.NUM_MULTIPLY
    NUM_ADD       = key.NUM_ADD
    NUM_SEPARATOR = key.NUM_SEPARATOR
    NUM_SUBTRACT  = key.NUM_SUBTRACT
    NUM_DECIMAL   = key.NUM_DECIMAL
    NUM_DIVIDE    = key.NUM_DIVIDE

    NUM_0         = key.NUM_0
    NUM_1         = key.NUM_1
    NUM_2         = key.NUM_2
    NUM_3         = key.NUM_3
    NUM_4         = key.NUM_4
    NUM_5         = key.NUM_5
    NUM_6         = key.NUM_6
    NUM_7         = key.NUM_7
    NUM_8         = key.NUM_8
    NUM_9         = key.NUM_9

    # Function keys
    F1            = key.F1
    F2            = key.F2
    F3            = key.F3
    F4            = key.F4
    F5            = key.F5
    F6            = key.F6
    F7            = key.F7
    F8            = key.F8
    F9            = key.F9
    F10           = key.F10
    F11           = key.F11
    F12           = key.F12
    F13           = key.F13
    F14           = key.F14
    F15           = key.F15
    F16           = key.F16
    F17           = key.F17
    F18           = key.F18
    F19           = key.F19
    F20           = key.F20
    F21           = key.F21
    F22           = key.F22
    F23           = key.F23
    F24           = key.F24
    # Modifiers
    LSHIFT        = key.LSHIFT
    RSHIFT        = key.RSHIFT
    LCTRL         = key.LCTRL
    RCTRL         = key.RCTRL
    CAPSLOCK      = key.CAPSLOCK
    LMETA         = key.LMETA
    RMETA         = key.RMETA
    LALT          = key.LALT
    RALT          = key.RALT
    LWINDOWS      = key.LWINDOWS
    RWINDOWS      = key.RWINDOWS
    LCOMMAND      = key.LCOMMAND
    RCOMMAND      = key.RCOMMAND
    LOPTION       = key.LOPTION
    ROPTION       = key.ROPTION

    # Latin-1
    SPACE         = key.SPACE
    EXCLAMATION   = key.EXCLAMATION
    DOUBLEQUOTE   = key.DOUBLEQUOTE
    HASH          = key.HASH
    POUND         = key.POUND  # synonym
    DOLLAR        = key.DOLLAR
    PERCENT       = key.PERCENT
    AMPERSAND     = key.AMPERSAND
    APOSTROPHE    = key.APOSTROPHE
    PARENLEFT     = key.PARENLEFT
    PARENRIGHT    = key.PARENRIGHT
    ASTERISK      = key.ASTERISK
    PLUS          = key.PLUS
    COMMA         = key.COMMA
    MINUS         = key.MINUS
    PERIOD        = key.PERIOD
    SLASH         = key.SLASH
    _0            = key._0
    _1            = key._1
    _2            = key._2
    _3            = key._3
    _4            = key._4
    _5            = key._5
    _6            = key._6
    _7            = key._7
    _8            = key._8
    _9            = key._9
    COLON         = key.COLON
    SEMICOLON     = key.SEMICOLON
    LESS          = key.LESS
    EQUAL         = key.EQUAL
    GREATER       = key.GREATER
    QUESTION      = key.QUESTION
    AT            = key.AT
    BRACKETLEFT   = key.BRACKETLEFT
    BACKSLASH     = key.BACKSLASH
    BRACKETRIGHT  = key.BRACKETRIGHT
    ASCIICIRCUM   = key.ASCIICIRCUM
    UNDERSCORE    = key.UNDERSCORE
    GRAVE         = key.GRAVE
    QUOTELEFT     = key.QUOTELEFT
    A             = key.A
    B             = key.B
    C             = key.C
    D             = key.D
    E             = key.E
    F             = key.F
    G             = key.G
    H             = key.H
    I             = key.I
    J             = key.J
    K             = key.K
    L             = key.L
    M             = key.M
    N             = key.N
    O             = key.O
    P             = key.P
    Q             = key.Q
    R             = key.R
    S             = key.S
    T             = key.T
    U             = key.U
    V             = key.V
    W             = key.W
    X             = key.X
    Y             = key.Y
    Z             = key.Z
    BRACELEFT     = key.BRACELEFT
    BAR           = key.BAR
    BRACERIGHT    = key.BRACERIGHT
    ASCIITILDE    = key.ASCIITILDE

class Mouse:   
    # MOUSE
    MOUSE1 = mouse.LEFT
    MOUSELEFT = mouse.LEFT       # synonym
    MOUSE2 = mouse.RIGHT
    MOUSERIGHT = mouse.RIGHT     # synonym
    MOUSE3 = mouse.MIDDLE
    MOUSEMIDDLE = mouse.MIDDLE   # synonym
    MOUSE4 = mouse.MOUSE4
    MOUSE5 = mouse.MOUSE5

class Color:
    # COLORS
    RED = (1, 0, 0, 1)
    BLUE = (0, 0, 1, 1)
    GREY = (0.4, 0.4, 0.4, 1)
    GRAY = (0.4, 0.4, 0.4, 1)  # synonym
    CYAN = (0, 1, 1, 1)
    WHITE = (1, 1, 1, 1)
    GREEN = (0, 1, 0, 1)
    BLACK = (0, 0, 0, 1)
    YELLOW = (1, 1, 0, 1)
    ORANGE = (1, 0.4, 0, 1)
    MAGENTA = (1, 0, 1, 1)
    DARKRED = (0.55, 0, 0, 1)
    DARKORANGE = (1, 0.55, 0, 1)
    DARKGREEN = (0, 0.4, 0, 1)
    YELLOWGREEN = (0.6, 0.8, 0.2, 1)
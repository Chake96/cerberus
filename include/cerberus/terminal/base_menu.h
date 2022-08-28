#ifndef __CERBERUS_TERMINAL_BASE_MENU_H_
#define __CERBERUS_TERMINAL_BASE_MENU_H_

namespace cerberus::terminal{

class BaseMenu{
    public:
        virtual ~BaseMenu() = default;
        virtual void start() = 0;
};

}//cerberus::terminal

#endif //__CERBERUS_TERMINAL_BASE_MENU_H_

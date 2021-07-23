#include "uav_com/UavCom.h"


const std::string UavCom::MASTER = "scout";
const std::string UavCom::SLAVE  = "bomber";


UavCom::UavCom(const def::BoardName& boardName) 
    : m_boardName(boardName)
    , m_nh("~")
{ }

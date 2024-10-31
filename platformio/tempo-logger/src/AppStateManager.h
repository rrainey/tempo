#ifndef APP_STATE_MANAGER_H
#define APP_STATE_MANAGER_H

class AppStateManager
{
    public:
    enum class APIResult {
            Success = 0,
            GenericError = -1,
            InitializationFailed = -2,
            SensorFault = -3
        };
    
    AppStateManager();

    /// @brief initialize application state to prepare for automatic detection of the
    ///        ascent for a jump.
    ///
    /// @return Success or InitializationFailed
    AppStateManager::APIResult setup();

    /// @brief call during each invocation of loop(); manage overall aplication state,
    ///        including creation of log files and logging of jump data.
    ///
    /// @return Success or GenericError
    AppStateManager::APIResult loop();

    protected:
    

};

#endif
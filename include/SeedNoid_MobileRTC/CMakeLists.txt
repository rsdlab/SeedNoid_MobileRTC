set(hdrs SeedNoid_MobileRTC.h SeedCommand.h SerialCom.h MecanumWheel.h
    PARENT_SCOPE
    SeedNoid_MobileSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)


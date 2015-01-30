# src/tools

ADD_SUBDIRECTORY( basestation )
ADD_SUBDIRECTORY( simulator/csim-0.1.0 )

ADD_CUSTOM_TARGET( tools DEPENDS
 basestation
)

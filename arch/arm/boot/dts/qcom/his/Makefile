ifeq ($(CONFIG_OF),y)

ifeq ($(subst \",,$(CONFIG_HIS_PRODUCT_NAME)),)
$(error "Please set CONFIG_HIS_PRODUCT_NAME value.")
endif

dtb-y += $(subst ",,$(CONFIG_HIS_PRODUCT_NAME)).dtb

DTB_LIST := $(dtb-y)

targets += dtbs
targets += $(addprefix ../../, $(DTB_LIST))
endif

$(obj)/../../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../../,$(DTB_LIST))

clean-files := *.dtb


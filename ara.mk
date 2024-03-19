
ARA_DIR=$(base_dir)/generators/ara/ara

ARA_FLIST = $(build_dir)/$(long_name).ara.f
ARA_INCDIRS = hardware/include hardware/deps/axi/include hardware/deps/common_cells/include hardware/deps/apb/include hardware/deps/ariane/common/local/util

ara_flist: $(ARA_FLIST)

$(ARA_FLIST): $(ARA_DIR)/../ara_files.f
	(cd $(ARA_DIR)/hardware && make update)
	-(cd $(ARA_DIR)/hardware && make apply-patches)
	awk '{print "$(ARA_DIR)/" $$0}' $^ > $@

USE_ARA ?=

ifeq ($(USE_ARA),1)
EXT_FILELISTS += $(ARA_FLIST)
EXT_INCDIRS += $(addprefix $(ARA_DIR)/,$(ARA_INCDIRS))
# this should not be a define....
SIM_PREPROC_DEFINES += +define+VLEN=4096
endif

#! /bin/bash

BOLD=$(tput bold)																			#加粗字体
NORMAL=$(tput sgr0)																			#正常字体
CUR_YEAR=$(date +"%Y")
TIZENRT_ROOT="$(dirname $0)/../.."															#获取当前脚本的路径(当前脚本位置的上两层)
cd ${TIZENRT_ROOT}
TIZENRT_ROOT="$(pwd)"
cd - > /dev/null																			#返回到上次目录,即脚本位置;将标准输出和标准出错的信息屏蔽不显示

echo "${BOLD}Nuttx Application Generator${NORMAL}"
echo "======================= v 1.0"

read -p "Enter application name: " APP_NAME      											#等待输入app的名称

APP_NAME_UPPER=$(echo "$APP_NAME" | tr '[:lower:]' '[:upper:]')								#将输入的app名称从小写转换为大写,存储在APP_NAME_UPPER
APP_NAME_UPPER="${APP_NAME_UPPER// /_}"														#将APP_NAME_UPPER中的空格替换为下划线
APP_NAME_LOWER=$(echo "$APP_NAME" | tr '[:upper:]' '[:lower:]')
APP_NAME_LOWER="${APP_NAME_LOWER// /_}"
ENTRY_FUNC="${APP_NAME_LOWER}_main"

echo "${BOLD}[Summary]${NORMAL}"
echo "-------------------------------"
echo "* Application Name: ${BOLD}${APP_NAME}${NORMAL}"
echo "* Configuration Key: ${BOLD}CONFIG_APP_${APP_NAME_UPPER}${NORMAL}"
echo "* Entry Function: ${BOLD}${ENTRY_FUNC}${NORMAL}"
echo "* Location: ${BOLD}${TIZENRT_ROOT}/apps/my_project/${APP_NAME_LOWER}${NORMAL}"
echo "* This year: ${BOLD}${CUR_YEAR}${NORMAL}"
echo "-------------------------------"
read -p "Continue? (y/N): " confirm && [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]] || exit 1

echo "Generating..."

KCONFIG_FILENAME="${TIZENRT_ROOT}/apps/my_project/${APP_NAME_LOWER}/Kconfig"
KCONFIG_ENTRY_FILENAME="${TIZENRT_ROOT}/apps/my_project/${APP_NAME_LOWER}/Kconfig_ENTRY"
MAIN_FILENAME="${TIZENRT_ROOT}/apps/my_project/${APP_NAME_LOWER}/${ENTRY_FUNC}.c"
MAKE_DEFS_FILENAME="${TIZENRT_ROOT}/apps/my_project/${APP_NAME_LOWER}/Make.defs"
MAKEFILE_FILENAME="${TIZENRT_ROOT}/apps/my_project/${APP_NAME_LOWER}/Makefile"

mkdir "${TIZENRT_ROOT}/apps/my_project/${APP_NAME_LOWER}"									#以输入的app小写名称创建文件夹
cp ${TIZENRT_ROOT}/tools/appgen/template_Kconfig ${KCONFIG_FILENAME}							#拷贝模版文件到对应目录,并修改名称....
#cp ${TIZENRT_ROOT}/tools/appgen/template_Kconfig_ENTRY ${KCONFIG_ENTRY_FILENAME}
cp ${TIZENRT_ROOT}/tools/appgen/template_main.c_source ${MAIN_FILENAME}
cp ${TIZENRT_ROOT}/tools/appgen/template_Make.defs ${MAKE_DEFS_FILENAME}
cp ${TIZENRT_ROOT}/tools/appgen/template_Makefile ${MAKEFILE_FILENAME}

sed -i -e "s/##YEAR##/${CUR_YEAR}/g" ${KCONFIG_FILENAME}
#sed -i -e "s/##YEAR##/${CUR_YEAR}/g" ${KCONFIG_ENTRY_FILENAME}
sed -i -e "s/##YEAR##/${CUR_YEAR}/g" ${MAIN_FILENAME}
sed -i -e "s/##YEAR##/${CUR_YEAR}/g" ${MAKE_DEFS_FILENAME}
sed -i -e "s/##YEAR##/${CUR_YEAR}/g" ${MAKEFILE_FILENAME}

sed -i -e "s/##APP_NAME_UPPER##/${APP_NAME_UPPER}/g" ${KCONFIG_FILENAME}
#sed -i -e "s/##APP_NAME_UPPER##/${APP_NAME_UPPER}/g" ${KCONFIG_ENTRY_FILENAME}
sed -i -e "s/##APP_NAME_UPPER##/${APP_NAME_UPPER}/g" ${MAIN_FILENAME}
sed -i -e "s/##APP_NAME_UPPER##/${APP_NAME_UPPER}/g" ${MAKE_DEFS_FILENAME}
sed -i -e "s/##APP_NAME_UPPER##/${APP_NAME_UPPER}/g" ${MAKEFILE_FILENAME}

sed -i -e "s/##APP_NAME_LOWER##/${APP_NAME_LOWER}/g" ${KCONFIG_FILENAME}
#sed -i -e "s/##APP_NAME_LOWER##/${APP_NAME_LOWER}/g" ${KCONFIG_ENTRY_FILENAME}
sed -i -e "s/##APP_NAME_LOWER##/${APP_NAME_LOWER}/g" ${MAIN_FILENAME}
sed -i -e "s/##APP_NAME_LOWER##/${APP_NAME_LOWER}/g" ${MAKE_DEFS_FILENAME}
sed -i -e "s/##APP_NAME_LOWER##/${APP_NAME_LOWER}/g" ${MAKEFILE_FILENAME}

sed -i -e "s/##APP_NAME##/${APP_NAME}/g" ${KCONFIG_FILENAME}
#sed -i -e "s/##APP_NAME##/${APP_NAME}/g" ${KCONFIG_ENTRY_FILENAME}
sed -i -e "s/##APP_NAME##/${APP_NAME}/g" ${MAIN_FILENAME}
sed -i -e "s/##APP_NAME##/${APP_NAME}/g" ${MAKE_DEFS_FILENAME}
sed -i -e "s/##APP_NAME##/${APP_NAME}/g" ${MAKEFILE_FILENAME}

sed -i -e "s/##ENTRY_FUNC##/${ENTRY_FUNC}/g" ${KCONFIG_FILENAME}
#sed -i -e "s/##ENTRY_FUNC##/${ENTRY_FUNC}/g" ${KCONFIG_ENTRY_FILENAME}
sed -i -e "s/##ENTRY_FUNC##/${ENTRY_FUNC}/g" ${MAIN_FILENAME}
sed -i -e "s/##ENTRY_FUNC##/${ENTRY_FUNC}/g" ${MAKE_DEFS_FILENAME}
sed -i -e "s/##ENTRY_FUNC##/${ENTRY_FUNC}/g" ${MAKEFILE_FILENAME}

echo ""
echo "* How to setup your application"
echo "Run) ${BOLD}nuttx\$${NORMAL} make menuconfig"
echo "1. Turn on your application in ${BOLD}Application Configuration/my_project${NORMAL} menu"
echo "2. Set the entry point to your application in ${BOLD}Application Configuration${NORMAL} menu"
echo "------------------------------"
echo "Done!!"


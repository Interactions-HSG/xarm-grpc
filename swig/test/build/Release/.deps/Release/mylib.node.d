cmd_Release/mylib.node := ln -f "Release/obj.target/mylib.node" "Release/mylib.node" 2>/dev/null || (rm -rf "Release/mylib.node" && cp -af "Release/obj.target/mylib.node" "Release/mylib.node")

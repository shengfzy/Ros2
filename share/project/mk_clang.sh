(
  find /opt/ros/humble/ -type d
  find /share/project/ -type d
) | sort -u | sed 's|//|/|g' | sed 's/^/    "-I/' | sed 's/$/",/' | awk 'BEGIN{print "CompileFlags:\n  Add: ["} {print} END{print "  ]"}' >/share/project/.clangd

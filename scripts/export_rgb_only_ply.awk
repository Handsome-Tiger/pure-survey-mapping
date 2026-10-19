BEGIN {
    in_header = 1
}

in_header && $0 == "property uchar intensity" {
    next
}

in_header {
    print
    if ($0 == "end_header") {
        in_header = 0
    }
    next
}

{
    print $1, $2, $3, $4, $5, $6
}

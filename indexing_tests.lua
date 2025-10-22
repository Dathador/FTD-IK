-- lua table indexing tests

function foo(table_index, table)
    return table[table_index]
end

function main()
    local table1 = {
        a = 1,
        b = 2,
    }

    print(foo("a", table1)) -- should print 1
    print(foo("b", table1)) -- should print 2
end

main()
cd(dirname(@__FILE__))

using Literate 

function replace_example_includes(str)

    included = ["seii_model.jl"]

    # Here the path loads the files from their proper directory,
    # which may not be the directory of the `examples.jl` file!
    path = "..\\"

    for ex in included
        content = read(path*ex, String)
        str = replace(str, "include(\"$(ex)\")" => content)
    end
    return str
end

Literate.markdown("..\\modelo_seii_input_desconocido.jl", "src\\ejemplos\\";
                  name = "NLFilterUnknownInput", preprocess = replace_example_includes#, 
                  #config = Dict("codefence" => ("```julia" => "```"))
                  )